#include <queue>
#include "path_planning/a_star_report.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <chrono>
#include <vector>
#include <unordered_set>
#include <iomanip> // Để format log

namespace path_planning
{

    void AStarPlanner_Report::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap(); 
        global_frame_ = costmap_ros->getGlobalFrameID();
        footprint_ = costmap_ros->getRobotFootprint(); //

        rclcpp::QoS smooth_qos(10);
        smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_.get(), "smooth_path");


        rclcpp::QoS plan_publisher_qos(rclcpp::KeepLast(1));
        plan_publisher_qos.transient_local(); 
        plan_publisher_qos.reliable(); 
    }

    void AStarPlanner_Report::activate() {}
    void AStarPlanner_Report::deactivate() {}
    void AStarPlanner_Report::cleanup() {}

    nav_msgs::msg::Path AStarPlanner_Report::createPlan(const geometry_msgs::msg::PoseStamped & start,
                                                const geometry_msgs::msg::PoseStamped & goal)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = node_->now();

        GraphNode start_node_grid = worldToGrid(start.pose);
        GraphNode goal_node_grid  = worldToGrid(goal.pose);

        if(!poseOnMap(start_node_grid) || !poseOnMap(goal_node_grid))
        {
            RCLCPP_ERROR(node_->get_logger(), "Start or goal pose out of map bounds.");
            return path;
        }

        // Định nghĩa kiểu cấu trúc để lưu kết quả của từng thuật toán
        enum class HeuristicType { MANHATTAN, EUCLIDEAN };
        struct SearchResult {
            std::string name;
            bool found{false};
            nav_msgs::msg::Path path;
            size_t nodes_expanded{0};
            double duration_ms{0.0};
            double path_length{0.0};
        };

        // Đóng gói lõi A* vào một hàm Lambda để chạy lại nhiều lần dễ dàng
        auto run_a_star = [&](HeuristicType h_type, const std::string& name) -> SearchResult {
            SearchResult result;
            result.name = name;
            auto t_start = std::chrono::high_resolution_clock::now();

            GraphNode s_node = start_node_grid;
            GraphNode g_node = goal_node_grid;

            auto cmp = [](const GraphNode & a, const GraphNode & b) {return a.totalCost() > b.totalCost();};
            std::priority_queue<GraphNode, std::vector<GraphNode>, decltype(cmp)> pending_nodes(cmp);
            std::unordered_set<unsigned int> visited_nodes;

            s_node.cost = 0;
            if (h_type == HeuristicType::MANHATTAN)
                s_node.heuristic = manhattanDistance(s_node, g_node);
            else
                s_node.heuristic = euclideanDistance(s_node, g_node);
            
            pending_nodes.push(s_node);
            GraphNode active_node;

            while(!pending_nodes.empty() && rclcpp::ok())
            {
                active_node = pending_nodes.top();
                pending_nodes.pop();
                result.nodes_expanded++;

                if(active_node.x == g_node.x && active_node.y == g_node.y) {
                    result.found = true;
                    break;
                }

                std::array<GraphNode, 4> neighbors = {
                    GraphNode(active_node.x + 1, active_node.y),
                    GraphNode(active_node.x - 1, active_node.y),
                    GraphNode(active_node.x, active_node.y + 1),
                    GraphNode(active_node.x, active_node.y - 1)
                };

                for(auto & new_node : neighbors)
                {
                    if(!poseOnMap(new_node)) continue;

                    auto cell_cost = costmap_->getCost(new_node.x, new_node.y);
                    if(cell_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) continue;

                    unsigned int cell_idx = poseToCell(new_node);
                    if(visited_nodes.find(cell_idx) == visited_nodes.end()) {
                        visited_nodes.insert(cell_idx);

                        auto move_cost = sqrt((active_node.x - new_node.x)* (active_node.x - new_node.x)
                                            + (active_node.y - new_node.y)* (active_node.y - new_node.y));

                        new_node.cost = active_node.cost + move_cost + costmap_->getCost(new_node.x, new_node.y);
                        
                        // Áp dụng Heuristic tương ứng
                        if (h_type == HeuristicType::MANHATTAN)
                            new_node.heuristic = manhattanDistance(new_node, g_node);
                        else
                            new_node.heuristic = euclideanDistance(new_node, g_node);

                        new_node.prev = std::make_shared<GraphNode>(active_node);
                        pending_nodes.push(new_node);
                    }
                }
            }

            auto t_end = std::chrono::high_resolution_clock::now();
            result.duration_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

            // Nếu tìm thấy, truy xuất đường đi
            if(result.found) {
                while(active_node.prev && rclcpp::ok()) {
                    geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
                    geometry_msgs::msg::PoseStamped last_pose_stamped;
                    last_pose_stamped.header.frame_id = global_frame_;
                    last_pose_stamped.header.stamp = node_->now();
                    last_pose_stamped.pose = last_pose;
                    result.path.poses.push_back(last_pose_stamped);
                    active_node = *active_node.prev;
                }
                std::reverse(result.path.poses.begin(), result.path.poses.end());
                result.path.header = path.header;

                // Tính toán chiều dài path
                for (size_t i = 1; i < result.path.poses.size(); ++i) {
                    const auto &p0 = result.path.poses[i - 1].pose.position;
                    const auto &p1 = result.path.poses[i].pose.position;
                    result.path_length += std::hypot(p1.x - p0.x, p1.y - p0.y);
                }
            }

            return result;
        };

        // ==========================================
        // THỰC THI VÀ SO SÁNH
        // ==========================================
        
        // 1. Chạy với Manhattan
        auto res_manhattan = run_a_star(HeuristicType::MANHATTAN, "Manhattan");
        
        // 2. Chạy với Euclidean
        auto res_euclidean = run_a_star(HeuristicType::EUCLIDEAN, "Euclidean");

        // 3. In bảng so sánh ra Console
        RCLCPP_INFO(node_->get_logger(), "\n================ BẢNG SO SÁNH HEURISTIC ================");
        RCLCPP_INFO(node_->get_logger(), "| %-12s | %-14s | %-12s | %-15s |", "Heuristic", "Nodes Expanded", "Time (ms)", "Path Length (m)");
        RCLCPP_INFO(node_->get_logger(), "-------------------------------------------------------------------");
        RCLCPP_INFO(node_->get_logger(), "| %-12s | %-14zu | %-12.3f | %-15.3f |", 
            res_manhattan.name.c_str(), res_manhattan.nodes_expanded, res_manhattan.duration_ms, res_manhattan.path_length);
        RCLCPP_INFO(node_->get_logger(), "| %-12s | %-14zu | %-12.3f | %-15.3f |", 
            res_euclidean.name.c_str(), res_euclidean.nodes_expanded, res_euclidean.duration_ms, res_euclidean.path_length);
        RCLCPP_INFO(node_->get_logger(), "===================================================================");

        // Quyết định trả về path nào (Ưu tiên Euclidean vì thường tạo đường chéo mượt hơn)
        if (!res_euclidean.found && !res_manhattan.found) {
            RCLCPP_WARN(node_->get_logger(), "No valid path found by any heuristic.");
            return path; // Path rỗng
        }

        // Chọn Euclidean làm đường chính, nếu fail thì lấy Manhattan
        path = res_euclidean.path_length < res_manhattan.path_length ? res_euclidean.path : res_manhattan.path; // Chọn heuristic nào có đường đi ngắn hơn

        // ==========================================
        // SMOOTHING STEP
        // ==========================================
        if(smooth_client_->action_server_is_ready()) {
            
            nav2_msgs::action::SmoothPath::Goal path_smooth;
            // --- Cleanup before smoothing ---
            {
              const double min_step = 1e-6;
              std::vector<geometry_msgs::msg::PoseStamped> cleaned;
              cleaned.reserve(path.poses.size());
              for (size_t i = 0; i < path.poses.size(); ++i) {
                if (cleaned.empty()) {
                  cleaned.push_back(path.poses[i]);
                } else {
                  auto &prev = cleaned.back().pose.position;
                  auto &cur  = path.poses[i].pose.position;
                  double dx = cur.x - prev.x;
                  double dy = cur.y - prev.y;
                  if (std::fabs(dx) + std::fabs(dy) > min_step) {
                    cleaned.push_back(path.poses[i]);
                  }
                }
              }
              path.poses.swap(cleaned);
            }

            path_smooth.path = path;
            path_smooth.check_for_collisions = false;
            path_smooth.smoother_id = "simple_smoother";
            path_smooth.max_smoothing_duration.sec = 10;    
            path_smooth.max_smoothing_duration.nanosec = 0; 

            auto goal_handle_future = smooth_client_->async_send_goal(path_smooth);
            if(goal_handle_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
            {
                auto goal_handle = goal_handle_future.get();
                if(goal_handle) {
                  auto result_future = smooth_client_->async_get_result(goal_handle);
                  if(result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
                  {
                    auto result_path = result_future.get();
                    if(result_path.result)
                    {
                        path = result_path.result->path;
                    }
                  }
                }
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "Smoother not available, returning raw path");
        }

        return path;
    }

    bool AStarPlanner_Report::poseOnMap(const GraphNode & node)
    {
        return node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.x >= 0 &&
               node.y < static_cast<int>(costmap_->getSizeInCellsY()) && node.y >= 0;
    }

    GraphNode AStarPlanner_Report::worldToGrid(const geometry_msgs::msg::Pose & pose)
    {
        int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
        int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
        return GraphNode(grid_x, grid_y);
    }

    geometry_msgs::msg::Pose AStarPlanner_Report::gridToWorld(const GraphNode & node)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
        pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
        return pose;
    }

    unsigned int AStarPlanner_Report::poseToCell(const GraphNode & node)
    {
        return costmap_->getSizeInCellsX() * node.y + node.x; 
    }

    double AStarPlanner_Report::manhattanDistance(const GraphNode & node, const GraphNode &goal_node)
    {
        return abs(node.x - goal_node.x) + abs(node.y - goal_node.y);
    }

    double AStarPlanner_Report::euclideanDistance(const GraphNode & node, const GraphNode &goal_node)
    {
        return sqrt(pow(node.x - goal_node.x, 2) + pow(node.y - goal_node.y, 2));
    }
}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(path_planning::AStarPlanner_Report, nav2_core::GlobalPlanner)