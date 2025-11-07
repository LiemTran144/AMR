#include <queue>
#include "path_planning/a_star_nav2_planner.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <vector>
#include <cmath>

static geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  return q;
}

namespace nhatbot_planning
{

    void AStarPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap(); 
        global_frame_ = costmap_ros->getGlobalFrameID();
        RCLCPP_INFO(
        node_->get_logger(),
        "Configuring %s of type AStarPlanner",
        name_.c_str());
        footprint_ = costmap_ros->getRobotFootprint();

    }

    void AStarPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating %s of type AStarPlanner", name_.c_str());
    }

    void AStarPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating %s of type AStarPlanner", name_.c_str());
    }

    void AStarPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up %s of type AStarPlanner", name_.c_str());
        node_.reset();
        costmap_ = nullptr;
    }

    nav_msgs::msg::Path AStarPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start,
                                                const geometry_msgs::msg::PoseStamped & goal)
    {
        // nav2_costmap_2d::Costmap2D * costmap_ = costmap_;
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;

        GraphNode start_node = worldToGrid(start.pose);
        GraphNode goal_node  = worldToGrid(goal.pose);

        if(!poseOnMap(start_node) || !poseOnMap(goal_node))
        {
            // RCLCPP_ERROR(logger_, "Start or goal pose out of map bounds.");
            return path;
        }

        auto cmp = [](const GraphNode & a, const GraphNode & b) {return a.totalCost() > b.totalCost();};
        std::priority_queue<GraphNode, std::vector<GraphNode>, decltype(cmp)> pending_nodes(cmp);
        std::vector<GraphNode> visited_nodes;

        start_node.cost = 0;
        start_node.heuristic = manhattanDistance(start_node, goal_node);
        pending_nodes.push(start_node);
        GraphNode active_node;
        bool found = false;

        while(!pending_nodes.empty() && rclcpp::ok())
        {
            active_node = pending_nodes.top();
            pending_nodes.pop();

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

            for(auto & new_node : neighbors)
            {
                if(!poseOnMap(new_node))
                    continue;

                auto cell_cost = costmap_->getCost(new_node.x, new_node.y);
                if(cell_cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
                    continue;

                bool visited = false;
                for(const auto & n : visited_nodes)
                {
                    if(n.x == new_node.x && n.y == new_node.y)
                    {
                        visited = true; break;
                    }
                }
                if(visited) continue;

                new_node.cost = active_node.cost + 1 + costmap_->getCost(new_node.x, new_node.y);
                new_node.heuristic = manhattanDistance(new_node, goal_node);
                new_node.prev = std::make_shared<GraphNode>(active_node);
                pending_nodes.push(new_node);
                visited_nodes.push_back(new_node);
            }
        }

        if(!found)
        {
            // RCLCPP_WARN(logger_, "No valid path found.");
            return path;
        }

        while(active_node.prev && rclcpp::ok())
        {
            geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
            geometry_msgs::msg::PoseStamped last_pose_stamped;
            last_pose_stamped.header.frame_id = global_frame_;
            last_pose_stamped.pose = last_pose;
            path.poses.push_back(last_pose_stamped);
            active_node = *active_node.prev;
        }
        std::reverse(path.poses.begin(), path.poses.end());

        // --- Cleanup: remove near-duplicate points and assign valid orientation ---
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

        if (!path.poses.empty()) {
          for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
            const auto &p0 = path.poses[i].pose.position;
            const auto &p1 = path.poses[i + 1].pose.position;
            double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
            path.poses[i].pose.orientation = yawToQuat(yaw);
          }
          if (path.poses.size() >= 2) {
            path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
          } else {
            path.poses.back().pose.orientation = yawToQuat(0.0);
          }
        }
        // --- End cleanup ---
        return path;
    }

    bool AStarPlanner::poseOnMap(const GraphNode & node)
    {
        return node.x < static_cast<int>(costmap_->getSizeInCellsX()) && node.x >= 0 &&
               node.y < static_cast<int>(costmap_->getSizeInCellsY()) && node.y >= 0;
    }

    GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
    {
        int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
        int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
        return GraphNode(grid_x, grid_y);
    }

    geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const GraphNode & node)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
        pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
        return pose;
    }

    unsigned int AStarPlanner::poseToCell(const GraphNode & node)
    {
        return costmap_->getSizeInCellsX() * node.y + node.x; 
    }

    double AStarPlanner::manhattanDistance(const GraphNode & node, const GraphNode &goal_node)
    {
        return abs(node.x - goal_node.x) + abs(node.y - goal_node.y);
    }
}  


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nhatbot_planning::AStarPlanner, nav2_core::GlobalPlanner)