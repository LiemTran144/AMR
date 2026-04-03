#include <queue>
#include "nhatbot_planner/liem_a_star_nav2_smoother.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <chrono>
#include <vector>
#include <unordered_set>
#include <iomanip> // Để format log
#include <fstream>

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
        // smooth_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(node_.get(), "smooth_path");

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

        // Định nghĩa kiểu cấu trúc để lưu kết quả của từng thuật toán (Thêm CHEBYSHEV)
        enum class HeuristicType { MANHATTAN, EUCLIDEAN, CHEBYSHEV, DJKSTRA };
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
            
            // Tính heuristic cho Node bắt đầu
            if (h_type == HeuristicType::MANHATTAN)
                s_node.heuristic = manhattanDistance(s_node, g_node);
            else if (h_type == HeuristicType::EUCLIDEAN)
                s_node.heuristic = euclideanDistance(s_node, g_node);
            else if (h_type == HeuristicType::CHEBYSHEV)
                s_node.heuristic = chebyshevDistance(s_node, g_node);
            else
                s_node.heuristic = 0.0; // Dijkstra
            
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

                // LUẬT DI CHUYỂN ĐỘNG: Phụ thuộc vào loại Heuristic
                std::vector<GraphNode> neighbors;
                neighbors.reserve(8); // Cấp phát sẵn bộ nhớ cho tối đa 8 node
                
                // 4 hướng cơ bản (Luôn luôn có)
                neighbors.push_back(GraphNode(active_node.x + 1, active_node.y));
                neighbors.push_back(GraphNode(active_node.x - 1, active_node.y));
                neighbors.push_back(GraphNode(active_node.x, active_node.y + 1));
                neighbors.push_back(GraphNode(active_node.x, active_node.y - 1));

                // Nếu KHÔNG PHẢI Manhattan, thêm 4 hướng chéo (Thành 8 hướng)
                if (h_type != HeuristicType::MANHATTAN) {
                    neighbors.push_back(GraphNode(active_node.x + 1, active_node.y + 1));
                    neighbors.push_back(GraphNode(active_node.x - 1, active_node.y + 1));
                    neighbors.push_back(GraphNode(active_node.x - 1, active_node.y - 1));
                    neighbors.push_back(GraphNode(active_node.x + 1, active_node.y - 1));
                }

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
                        
                        // Áp dụng Heuristic tương ứng cho Node lân cận
                        if (h_type == HeuristicType::MANHATTAN)
                            new_node.heuristic = manhattanDistance(new_node, g_node);
                        else if (h_type == HeuristicType::EUCLIDEAN)
                            new_node.heuristic = euclideanDistance(new_node, g_node);
                        else if (h_type == HeuristicType::CHEBYSHEV)
                            new_node.heuristic = chebyshevDistance(new_node, g_node);
                        else
                            new_node.heuristic = 0.0; // Dijkstra

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
        
        // 1. Chạy với Manhattan (sẽ tự động dùng 4 hướng)
        auto res_manhattan = run_a_star(HeuristicType::MANHATTAN, "Manhattan(4D)");
        
        // 2. Chạy với Euclidean (sẽ tự động dùng 8 hướng)
        auto res_euclidean = run_a_star(HeuristicType::EUCLIDEAN, "Euclidean(8D)");
        
        // 3. Chạy với Chebyshev (sẽ tự động dùng 8 hướng)
        auto res_chebyshev = run_a_star(HeuristicType::CHEBYSHEV, "Chebyshev(8D)");

        // 4. Chạy với Dijkstra (sẽ tự động dùng 8 hướng)
        auto res_dijkstra = run_a_star(HeuristicType::DJKSTRA, "Dijkstra(8D)"); 

        // 5. In bảng so sánh ra Console
        RCLCPP_INFO(node_->get_logger(), "\n================ BẢNG SO SÁNH HEURISTIC ================");
        RCLCPP_INFO(node_->get_logger(), "| %-14s | %-14s | %-12s | %-15s |", "Heuristic", "Nodes Expanded", "Time (ms)", "Path Length (m)");
        RCLCPP_INFO(node_->get_logger(), "---------------------------------------------------------------------");
        RCLCPP_INFO(node_->get_logger(), "| %-14s | %-14zu | %-12.3f | %-15.3f |", 
            res_manhattan.name.c_str(), res_manhattan.nodes_expanded, res_manhattan.duration_ms, res_manhattan.path_length);
        RCLCPP_INFO(node_->get_logger(), "| %-14s | %-14zu | %-12.3f | %-15.3f |", 
            res_euclidean.name.c_str(), res_euclidean.nodes_expanded, res_euclidean.duration_ms, res_euclidean.path_length);
        RCLCPP_INFO(node_->get_logger(), "| %-14s | %-14zu | %-12.3f | %-15.3f |", 
            res_chebyshev.name.c_str(), res_chebyshev.nodes_expanded, res_chebyshev.duration_ms, res_chebyshev.path_length);
        RCLCPP_INFO(node_->get_logger(), "| %-14s | %-14zu | %-12.3f | %-15.3f |", 
            res_dijkstra.name.c_str(), res_dijkstra.nodes_expanded, res_dijkstra.duration_ms, res_dijkstra.path_length);
        RCLCPP_INFO(node_->get_logger(), "=====================================================================");

        if (!res_euclidean.found && !res_manhattan.found && !res_chebyshev.found && !res_dijkstra.found) {
            RCLCPP_WARN(node_->get_logger(), "No valid path found by any heuristic.");
            return path; // Path rỗng
        }
        
        // Chọn heuristic có đường đi ngắn nhất để trả về (Bỏ qua Dijkstra vì nó quá chậm, chỉ để benchmark)
        path = res_euclidean.path_length < res_manhattan.path_length ? res_euclidean.path : res_manhattan.path; 

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
    
    double AStarPlanner_Report::chebyshevDistance(const GraphNode & node, const GraphNode &goal_node)
    {
        return std::max(abs(node.x - goal_node.x), abs(node.y - goal_node.y));
    }
}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(path_planning::AStarPlanner_Report, nav2_core::GlobalPlanner)