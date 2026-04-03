#include <queue>
#include "nhatbot_planner/liem_astar_present.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <chrono>
#include <vector>
#include <unordered_set>

namespace path_planning
{

    void AStarPlanner_Present::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
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

        // raw_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("raw_plan", plan_publisher_qos);
        // ---------------------
    }

    void AStarPlanner_Present::activate() {}
    void AStarPlanner_Present::deactivate() {}
    void AStarPlanner_Present::cleanup() {}

    nav_msgs::msg::Path AStarPlanner_Present::createPlan(const geometry_msgs::msg::PoseStamped & start,
                                                const geometry_msgs::msg::PoseStamped & goal)
    {
        auto t_plan_start = std::chrono::high_resolution_clock::now();

        // nav2_costmap_2d::Costmap2D * costmap_ = costmap_;
        GraphNode start_node = worldToGrid(start.pose);
        GraphNode goal_node  = worldToGrid(goal.pose);

        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = node_->now();

        if(!poseOnMap(start_node) || !poseOnMap(goal_node))
        {
            RCLCPP_ERROR(node_->get_logger(), "Start or goal pose out of map bounds.");
            return path;
        }

        auto cmp = [](const GraphNode & a, const GraphNode & b) {return a.totalCost() > b.totalCost();};
        std::priority_queue<GraphNode, std::vector<GraphNode>, decltype(cmp)> pending_nodes(cmp); // open list 
        std::unordered_set<unsigned int> visited_nodes;  // closed list 

        start_node.cost = 0;
        start_node.heuristic = manhattanDistance(start_node, goal_node);
        // start_node.heuristic = euclideanDistance(start_node, goal_node);
        pending_nodes.push(start_node);
        GraphNode active_node;
        bool found = false;
        size_t nodes_expanded = 0;

        while(!pending_nodes.empty() && rclcpp::ok())
        {
            active_node = pending_nodes.top();
            pending_nodes.pop();
            nodes_expanded++;

            if(active_node.x == goal_node.x && active_node.y == goal_node.y)
            {
                found = true;
                break;
            }

            std::array<GraphNode, 4> neighbors = {
                GraphNode(active_node.x + 1, active_node.y),
                GraphNode(active_node.x - 1, active_node.y),
                GraphNode(active_node.x, active_node.y + 1),
                GraphNode(active_node.x, active_node.y - 1)
            };

            // std::array<GraphNode, 8> neighbors = {
            //     GraphNode(active_node.x + 1, active_node.y),
            //     GraphNode(active_node.x + 1, active_node.y + 1),
            //     GraphNode(active_node.x, active_node.y + 1),
            //     GraphNode(active_node.x - 1, active_node.y + 1),
            //     GraphNode(active_node.x - 1, active_node.y),
            //     GraphNode(active_node.x - 1, active_node.y - 1),
            //     GraphNode(active_node.x, active_node.y - 1),
            //     GraphNode(active_node.x + 1, active_node.y - 1)
            // };

            for(auto & new_node : neighbors)
            {
                if(!poseOnMap(new_node))
                    continue;

                auto cell_cost = costmap_->getCost(new_node.x, new_node.y);
                if(cell_cost >= nav2_costmap_2d::LETHAL_OBSTACLE)  
                    continue;

                unsigned int cell_idx = poseToCell(new_node);
                if(visited_nodes.find(cell_idx) == visited_nodes.end()) {
                    visited_nodes.insert(cell_idx);

                    auto move_cost = sqrt((active_node.x - new_node.x)* (active_node.x - new_node.x)
                                        + (active_node.y - new_node.y)* (active_node.y - new_node.y));

                    new_node.cost = active_node.cost + move_cost + costmap_->getCost(new_node.x, new_node.y);
                    new_node.heuristic = manhattanDistance(new_node, goal_node);
                    // new_node.heuristic = euclideanDistance(new_node, goal_node);
                    new_node.prev = std::make_shared<GraphNode>(active_node);
                    pending_nodes.push(new_node);
                }
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Expanded %zu nodes to reach goal, closed set size %zu, open set size %zu", nodes_expanded, visited_nodes.size(), pending_nodes.size());

        if(!found)
        {
            RCLCPP_WARN(node_->get_logger(), "No valid path found.");
            return path;
        }

        while(active_node.prev && rclcpp::ok())
        {
            geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
            geometry_msgs::msg::PoseStamped last_pose_stamped;
            last_pose_stamped.header.frame_id = global_frame_;
            last_pose_stamped.header.stamp = node_->now();
            last_pose_stamped.pose = last_pose;
            path.poses.push_back(last_pose_stamped);
            active_node = *active_node.prev;
        }
        std::reverse(path.poses.begin(), path.poses.end());

        // raw_path_pub_->publish(path);  // Publish raw path before smoothing

        // if (raw_path_pub_->get_subscription_count() > 0) {
        //     raw_path_pub_->publish(path);
        // }

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

        // Compute and log path length in meters
        double path_length_sum = 0.0;
        for (size_t i = 1; i < path.poses.size(); ++i) {
            const auto &p0 = path.poses[i - 1].pose.position;
            const auto &p1 = path.poses[i].pose.position;
            double dx = p1.x - p0.x;
            double dy = p1.y - p0.y;
            path_length_sum += std::hypot(dx, dy);
        }

        // // Line integral style distance (approximation by segment sampling)
        // double path_length_line_integral = 0.0;
        // const size_t samples_per_segment = 10;
        // for (size_t i = 1; i < path.poses.size(); ++i) {
        //     const auto &p0 = path.poses[i - 1].pose.position;
        //     const auto &p1 = path.poses[i].pose.position;
        //     double dx = p1.x - p0.x;
        //     double dy = p1.y - p0.y;
        //     for (size_t k = 1; k <= samples_per_segment; ++k) {
        //         double t_prev = static_cast<double>(k - 1) / samples_per_segment;
        //         double t = static_cast<double>(k) / samples_per_segment;
        //         double x_prev = p0.x + t_prev * dx;
        //         double y_prev = p0.y + t_prev * dy;
        //         double x_cur = p0.x + t * dx;
        //         double y_cur = p0.y + t * dy;
        //         path_length_line_integral += std::hypot(x_cur - x_prev, y_cur - y_prev);
        //     }
        // }

        // RCLCPP_INFO(node_->get_logger(), "Path length (sum): %.3f m, line integral approx: %.3f m, waypoints: %zu", path_length_sum, path_length_line_integral, path.poses.size());
        auto t_plan_end = std::chrono::high_resolution_clock::now();
        double total_ms = std::chrono::duration<double, std::milli>(t_plan_end - t_plan_start).count();

        RCLCPP_INFO(node_->get_logger(), "Expanded %zu nodes to reach goal, closed set size %zu, open set size %zu", nodes_expanded, visited_nodes.size(), pending_nodes.size());
        RCLCPP_INFO(node_->get_logger(), "Path length (sum): %.3f m", path_length_sum);
        RCLCPP_INFO(node_->get_logger(), "Total duration: %.3f ms", total_ms);

        return path;
    }

    bool AStarPlanner_Present::poseOnMap(const GraphNode & node)
    {
        return node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.x >= 0 &&
               node.y < static_cast<int>(costmap_->getSizeInCellsY()) && node.y >= 0;
    }

    GraphNode AStarPlanner_Present::worldToGrid(const geometry_msgs::msg::Pose & pose)
    {
        int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
        int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
        return GraphNode(grid_x, grid_y);
    }

    geometry_msgs::msg::Pose AStarPlanner_Present::gridToWorld(const GraphNode & node)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
        pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
        return pose;
    }

    unsigned int AStarPlanner_Present::poseToCell(const GraphNode & node)
    {
        return costmap_->getSizeInCellsX() * node.y + node.x; 
    }

    double AStarPlanner_Present::manhattanDistance(const GraphNode & node, const GraphNode &goal_node)
    {
        return abs(node.x - goal_node.x) + abs(node.y - goal_node.y);
    }

    double AStarPlanner_Present::euclideanDistance(const GraphNode & node, const GraphNode &goal_node)
    {
        return sqrt(pow(node.x - goal_node.x, 2) + pow(node.y - goal_node.y, 2));
    }
}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(path_planning::AStarPlanner_Present, nav2_core::GlobalPlanner)