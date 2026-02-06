#ifndef A_STAR_PLANNER_HPP
#define A_STAR_PLANNER_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"


namespace nhatbot_planning
{
    struct GraphNode
    {
        int x{0};
        int y{0};
        double cost{0.0};
        double heuristic{0.0};
        std::shared_ptr<GraphNode> prev{nullptr};
        GraphNode() = default;
        GraphNode(int xi, int yi) : x(xi), y(yi) {}
        inline double totalCost() const { return cost + heuristic; }
    };

    class AStarPlanner : public nav2_core::GlobalPlanner
    {
    public:
        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                       std::shared_ptr<tf2_ros::Buffer> tf,
                       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                                       const geometry_msgs::msg::PoseStamped & goal) override;

    private:
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        // nav2_costmap_2d::Costmap2D * costmap_{nullptr};
        std::vector<geometry_msgs::msg::Point> footprint_;
        std::string global_frame_;
        std::string name_;
        nav2_costmap_2d::Costmap2D* costmap_ = nullptr;

        bool poseOnMap(const GraphNode & node);
        GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
        geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
        unsigned int poseToCell(const GraphNode & node);
        double manhattanDistance(const GraphNode & node, const GraphNode & goal_node);
    };
}  // namespace nhatbot_planning


#endif 
