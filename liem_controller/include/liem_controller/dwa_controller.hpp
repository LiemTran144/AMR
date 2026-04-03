#ifndef DWA_CONTROLLER__DWA_CONTROLLER_HPP_
#define DWA_CONTROLLER__DWA_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>
#include <tuple>
#include <cmath>

// ROS 2 Core
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

// Nav2 Interfaces & Utils
#include "nav2_core/controller.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// TF2
#include "tf2_ros/buffer.h"
#include "angles/angles.h"

namespace dwa_controller
{

enum class ControlState {
  INITIAL_CHECK, // Vừa nhận plan mới, cần kiểm tra góc lệch
  ROTATING,      // Quyết định xoay tại chỗ
  FOLLOWING      // Đã thẳng hàng, chạy DWA
};

class DWA_Controller : public nav2_core::Controller
{
public:
  DWA_Controller() = default;
  ~DWA_Controller() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  
  // Reset trạng thái về INITIAL_CHECK khi có plan mới
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

protected:
  std::vector<geometry_msgs::msg::PoseStamped> transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);

  std::tuple<double, double, double, double> computeDynamicWindow(
    const geometry_msgs::msg::Twist & current_vel);

  std::vector<geometry_msgs::msg::PoseStamped> simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const double v,
    const double w);

  double checkTrajectoryCollision(
    const std::vector<geometry_msgs::msg::PoseStamped> & trajectory);

  double scoreTrajectory(
    const double v, 
    const double w, 
    const std::vector<geometry_msgs::msg::PoseStamped> & local_plan,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & final_goal_pose );

  bool rotateToHeading(
    double target_yaw, 
    const geometry_msgs::msg::PoseStamped & current_pose,
    geometry_msgs::msg::TwistStamped & cmd_vel,
    double tolerance);

  // void publish_trajectories (const std::vector<std::vector<geometry_msgs::msg::PoseStamped>>& trajectories);

  // --- VARIABLES ---
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("DWA_Controller")};

  // Biến lưu trạng thái hiện tại
  ControlState current_state_ = ControlState::INITIAL_CHECK;

  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> marker_pub_;

  nav_msgs::msg::Path global_plan_;

  // Limits
  double max_vel_x_, min_vel_x_;
  double max_vel_theta_, min_vel_theta_;
  double min_in_place_vel_theta_;
  double acc_lim_x_, acc_lim_theta_;
  
  // Simulation
  double sim_time_;
  double sim_granularity_;
  double controller_frequency_;

  // Scoring
  double scale_heading_; 
  double scale_obs_;     
  double scale_vel_;     
  double scale_align_;   

  // Tolerances
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double rotate_to_heading_min_angle_;

  std::string robot_base_frame_;
  std::string global_frame_;
};

} // namespace dwa_controller

#endif // DWA_CONTROLLER__DWA_CONTROLLER_HPP_