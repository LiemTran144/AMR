#include "liem_controller/dwa_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_core/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "nav2_util/geometry_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace dwa_controller
{

void DWA_Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, 
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();

  RCLCPP_INFO(logger_, "Configuring DWA Controller: %s", plugin_name_.c_str());

  declare_parameter_if_not_declared(node, plugin_name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_vel_x", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_vel_theta", rclcpp::ParameterValue(-1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_in_place_vel_theta", rclcpp::ParameterValue(0.4));
  
  declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lim_x", rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.2));
  
  declare_parameter_if_not_declared(node, plugin_name_ + ".sim_time", rclcpp::ParameterValue(1.7));
  declare_parameter_if_not_declared(node, plugin_name_ + ".sim_granularity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".controller_frequency", rclcpp::ParameterValue(20.0));
  
  declare_parameter_if_not_declared(node, plugin_name_ + ".scale_heading", rclcpp::ParameterValue(32.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".scale_obs", rclcpp::ParameterValue(24.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".scale_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".scale_align", rclcpp::ParameterValue(10.0));

  declare_parameter_if_not_declared(node, plugin_name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785)); // 45 do
  declare_parameter_if_not_declared(node, plugin_name_ + ".robot_base_frame", rclcpp::ParameterValue("base_link"));

  node->get_parameter(plugin_name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(plugin_name_ + ".min_vel_x", min_vel_x_);
  node->get_parameter(plugin_name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(plugin_name_ + ".min_vel_theta", min_vel_theta_);
  node->get_parameter(plugin_name_ + ".min_in_place_vel_theta", min_in_place_vel_theta_);

  node->get_parameter(plugin_name_ + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(plugin_name_ + ".acc_lim_theta", acc_lim_theta_);
  
  node->get_parameter(plugin_name_ + ".sim_time", sim_time_);
  node->get_parameter(plugin_name_ + ".sim_granularity", sim_granularity_);
  node->get_parameter(plugin_name_ + ".controller_frequency", controller_frequency_);
  
  node->get_parameter(plugin_name_ + ".scale_heading", scale_heading_);
  node->get_parameter(plugin_name_ + ".scale_obs", scale_obs_);
  node->get_parameter(plugin_name_ + ".scale_vel", scale_vel_);
  node->get_parameter(plugin_name_ + ".scale_align", scale_align_);

  node->get_parameter(plugin_name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  
  node->get_parameter(plugin_name_ + ".robot_base_frame", robot_base_frame_);
  
  global_frame_ = costmap_ros_->getGlobalFrameID();

  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 1);
}

void DWA_Controller::activate()
{
  RCLCPP_INFO(logger_, "Activating DWA Controller");
  local_plan_pub_->on_activate();
  marker_pub_->on_activate();
  current_state_ = ControlState::INITIAL_CHECK; 
}

void DWA_Controller::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating DWA Controller");
  local_plan_pub_->on_deactivate();
  marker_pub_->on_deactivate();
}

void DWA_Controller::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up DWA Controller");
  local_plan_pub_.reset();
  marker_pub_.reset();
  collision_checker_.reset();
}

void DWA_Controller::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_vel_x_ *= speed_limit;
    min_vel_x_ *= speed_limit;
    max_vel_theta_ *= speed_limit;
    min_vel_theta_ *= speed_limit;
  } else {
    max_vel_x_ = speed_limit;
    if (min_vel_x_ > max_vel_x_) {
        min_vel_x_ = 0.0;
    }
  }
}

void DWA_Controller::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  current_state_ = ControlState::INITIAL_CHECK;
  RCLCPP_INFO(logger_, "New plan received. State reset to INITIAL_CHECK.");
}

geometry_msgs::msg::TwistStamped DWA_Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.header.stamp = rclcpp::Clock().now();

  if (global_plan_.poses.empty()) return cmd_vel;

  // 1. Goal Checker 
  if (goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    RCLCPP_INFO(logger_, "GOAL REACHED! Stopping.");
    return cmd_vel;
  }

  // // Get Robot Pose
  geometry_msgs::msg::PoseStamped current_robot_pose; 
  if (!costmap_ros_->getRobotPose(current_robot_pose))  
  {
      return cmd_vel;
  }
  
  geometry_msgs::msg::PoseStamped final_goal_pose = global_plan_.poses.back();
  try {
    // Chuyển đổi điểm đích cuối cùng sang global_frame_ (thường là odom hoặc map)
    geometry_msgs::msg::TransformStamped transform = tf_->lookupTransform(
      global_frame_, global_plan_.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(final_goal_pose, final_goal_pose, transform);


  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Goal transform failed: %s", ex.what());
    return cmd_vel;
  }


  // double dist_to_goal = std::hypot(
  //     last_pose.pose.position.x - final_goal_pose.pose.position.x,
  //     last_pose.pose.position.y - final_goal_pose.pose.position.y);

  // if (dist_to_goal < xy_goal_tolerance_) {
  //     cmd_vel.twist.linear.x = 0.0;
  //     cmd_vel.twist.angular.z = 0.0;
  //     RCLCPP_INFO(logger_, "Reach Goal!");
  //     return cmd_vel;
  // }

  // Transform Plan
  std::vector<geometry_msgs::msg::PoseStamped> local_plan;
  try {
    local_plan = transformGlobalPlan(current_robot_pose);
  } catch (const nav2_core::PlannerException & e) {
    RCLCPP_ERROR(logger_, "Could not transform plan: %s", e.what());
    return cmd_vel;                                
  }

  // if (local_plan_pub_->get_subscription_count() > 0 && !local_plan.empty()) {
  if (!local_plan.empty()) {
    nav_msgs::msg::Path local_path;
    local_path.header = local_plan.front().header;
    local_path.poses = local_plan;
    local_plan_pub_->publish(local_path);
  }


  double heading_error;
  double target_yaw;

  if (!local_plan.empty()) {
      double lookahead_x = local_plan.front().pose.position.x - current_robot_pose.pose.position.x;
      double lookahead_y = local_plan.front().pose.position.y - current_robot_pose.pose.position.y;

      if (std::hypot(lookahead_x, lookahead_y) < 0.1 && local_plan.size() > 5) {
          lookahead_x = local_plan[5].pose.position.x - current_robot_pose.pose.position.x;
          lookahead_y = local_plan[5].pose.position.y - current_robot_pose.pose.position.y;
      }
      target_yaw = std::atan2(lookahead_y, lookahead_x);
      double current_yaw = tf2::getYaw(current_robot_pose.pose.orientation);
      heading_error = angles::shortest_angular_distance(current_yaw, target_yaw);
  }

  if (current_state_ == ControlState::INITIAL_CHECK) {
      if (std::abs(heading_error) > rotate_to_heading_min_angle_) {
          current_state_ = ControlState::ROTATING;
          RCLCPP_INFO(logger_, "Heading error %.2f > Threshold. Switching to ROTATING.", heading_error);
      } else {
          current_state_ = ControlState::FOLLOWING;
          RCLCPP_INFO(logger_, "Heading error %.2f is small. Skipping rotation. Switching to FOLLOWING.", heading_error);
      }
  }



  if (current_state_ == ControlState::ROTATING) {
      if (std::abs(heading_error) < yaw_goal_tolerance_) {
          current_state_ = ControlState::FOLLOWING;
          RCLCPP_INFO(logger_, "Aligned (Error %.2f). Switching to FOLLOWING (DWA).", heading_error);
      }
  }

  
  // CASE: ROTATING
  if (current_state_ == ControlState::ROTATING) {
      if (rotateToHeading(target_yaw, current_robot_pose, cmd_vel, yaw_goal_tolerance_)) {
          return cmd_vel;
      } else {

          RCLCPP_WARN(logger_, "Rotation blocked by collision! Forcing switch to FOLLOWING.");
          current_state_ = ControlState::FOLLOWING;
      }
  }

  // CASE: FOLLOWING 
  if (current_state_ == ControlState::FOLLOWING) {
      auto [min_v, max_v, min_w, max_w] = computeDynamicWindow(velocity);

      double best_score = -1e9;
      double best_v = 0.0;
      double best_w = 0.0;
      bool found_valid_traj = false;

      double v_sample_step = 0.05; 
      double w_sample_step = 0.05;
      
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

      for (double v = min_v; v <= max_v; v += v_sample_step) {
        for (double w = min_w; w <= max_w; w += w_sample_step) {
          double score = scoreTrajectory(v, w, local_plan, current_robot_pose, final_goal_pose);
          if (score < 0.0) continue; 
          if (score > best_score) {
            best_score = score;
            best_v = v;
            best_w = w;
            found_valid_traj = true;
          }
        }
      }

      if (found_valid_traj) {
        cmd_vel.twist.linear.x = best_v;
        cmd_vel.twist.angular.z = best_w;

      } else {
        RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 2000, 
          "DWA failed! Robot stops.");
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
      }
  }

  return cmd_vel;
}

bool DWA_Controller::rotateToHeading(
    double target_yaw, 
    const geometry_msgs::msg::PoseStamped & current_pose,
    geometry_msgs::msg::TwistStamped & cmd_vel,
    double tolerance)
{
    double current_yaw = tf2::getYaw(current_pose.pose.orientation);
    double diff = angles::shortest_angular_distance(current_yaw, target_yaw);

    if (std::abs(diff) < tolerance) {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return true; 
    }

    double sign = (diff > 0) ? 1.0 : -1.0;
    double vel = sign * std::min(std::abs(diff) * 0.6, max_vel_theta_); 
    
    if (std::abs(vel) < min_in_place_vel_theta_) {
        vel = sign * min_in_place_vel_theta_;
    }

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    auto traj = simulateTrajectory(current_pose, 0.0, vel); 
    double cost = checkTrajectoryCollision(traj);

    if (cost < 0.0) { 
        return false; 
    }

    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = vel;
    return true;
}

std::vector<geometry_msgs::msg::PoseStamped> DWA_Controller::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  
  geometry_msgs::msg::TransformStamped plan_to_costmap_transform;
  try {
    plan_to_costmap_transform = tf_->lookupTransform(
      costmap_frame,
      global_plan_.header.frame_id,
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    throw nav2_core::PlannerException("Could not transform plan: " + std::string(ex.what()));
  }

  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  
  for (const auto & global_pose : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped local_pose;
    tf2::doTransform(global_pose, local_pose, plan_to_costmap_transform);
    transformed_plan.push_back(local_pose);
  }

  double min_dist = 1e9;
  size_t closest_index = 0;
  for (size_t i = 0; i < transformed_plan.size(); ++i) {
    double d = std::hypot(
      pose.pose.position.x - transformed_plan[i].pose.position.x,
      pose.pose.position.y - transformed_plan[i].pose.position.y);
    if (d < min_dist) {
      min_dist = d;
      closest_index = i;
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> final_plan;
  size_t lookahead_dist = 50; 
  for (size_t i = closest_index; i < transformed_plan.size() && i < closest_index + lookahead_dist; ++i) {
    final_plan.push_back(transformed_plan[i]);
  }

  if (final_plan.empty()) return transformed_plan; 
  return final_plan;
}

std::tuple<double, double, double, double> DWA_Controller::computeDynamicWindow(
  const geometry_msgs::msg::Twist & current_vel)
{
  double dt = 1.0 / controller_frequency_;

  double min_v_acc = current_vel.linear.x - acc_lim_x_ * dt;
  double max_v_acc = current_vel.linear.x + acc_lim_x_ * dt;
  double min_v = std::max(min_vel_x_, min_v_acc);
  double max_v = std::min(max_vel_x_, max_v_acc); 

  double min_w_acc = current_vel.angular.z - acc_lim_theta_ * dt;
  double max_w_acc = current_vel.angular.z + acc_lim_theta_ * dt;
  double min_w = std::max(min_vel_theta_, min_w_acc);
  double max_w = std::min(max_vel_theta_, max_w_acc);

  return std::make_tuple(min_v, max_v, min_w, max_w);
}

std::vector<geometry_msgs::msg::PoseStamped> DWA_Controller::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const double v,
  const double w)
{
  std::vector<geometry_msgs::msg::PoseStamped> trajectory;
  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double theta = tf2::getYaw(current_pose.pose.orientation);

  double dt = sim_granularity_;
  int steps = std::ceil(sim_time_ / dt);

  for (int i = 0; i < steps; ++i) {
    x += v * cos(theta) * dt;
    y += v * sin(theta) * dt;
    theta += w * dt;

    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = current_pose.header.frame_id;
    p.header.stamp = current_pose.header.stamp; 
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    p.pose.orientation = tf2::toMsg(q);

    trajectory.push_back(p);
  }
  return trajectory;
}

double DWA_Controller::checkTrajectoryCollision(
  const std::vector<geometry_msgs::msg::PoseStamped> & trajectory)
{
  if (trajectory.empty()) return 0.0;

  std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();
  
  double max_cost = 0.0;

  for (const auto & pose : trajectory) {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double theta = tf2::getYaw(pose.pose.orientation);

    double cost = collision_checker_->footprintCostAtPose(x, y, theta, footprint);

    if (cost == nav2_costmap_2d::NO_INFORMATION) {
        continue; 
    }

    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      return -1.0; 
    }

    if (cost > max_cost) max_cost = cost;
  }
  return max_cost;
}


double DWA_Controller::scoreTrajectory(
  const double v, 
  const double w, 
  const std::vector<geometry_msgs::msg::PoseStamped> & local_plan,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & final_goal_pose)
{
  
  auto trajectory = simulateTrajectory(current_pose, v, w);

  // RCLCPP_INFO_THROTTLE(logger_, *node_.lock()->get_clock(), 1000,
  //   "Simulated Trajectory for v=%.2f m/s, w=%.2f rad/s with %zu poses.",
  //   v, w, trajectory.size());
  // publish_trajectories({trajectory});


  // 1. Obstacle Cost
  double obs_raw_cost = checkTrajectoryCollision(trajectory);
  if (obs_raw_cost < 0.0) return -1.0; 

  double normalized_obs_cost = obs_raw_cost / 254.0; 

  auto last_pose = trajectory.back();

  // 2. Alignment Cost
  double min_dist_to_path = 1e9;
  for (const auto & path_pose : local_plan) {
    double d = std::hypot(
      last_pose.pose.position.x - path_pose.pose.position.x,
      last_pose.pose.position.y - path_pose.pose.position.y);
    if (d < min_dist_to_path) min_dist_to_path = d;
  }
  
  // 3. Goal Cost 
  double dist_to_goal = std::hypot(
      last_pose.pose.position.x - final_goal_pose.pose.position.x,
      last_pose.pose.position.y - final_goal_pose.pose.position.y);

  // Compute Utility (Higher is better)
  double utility_obs = 1.0 - normalized_obs_cost;
  double utility_align = 1.0 / (1.0 + min_dist_to_path); 
  double utility_goal = 1.0 / (1.0 + dist_to_goal);      
  double utility_vel = v / max_vel_x_;                   

  double total_score = (scale_obs_ * utility_obs) + 
                       (scale_align_ * utility_align) + 
                       (scale_heading_ * utility_goal) + 
                       (scale_vel_ * utility_vel);

  RCLCPP_INFO_THROTTLE(logger_, *node_.lock()->get_clock(), 1000,
    "Window v:[%.2f, %.2f] w:[%.2f, %.2f] Traj (v=%.2f, w=%.2f): Score=%.2f [Obs:%.2f(%.0f), Align:%.2f(%.2fm), Goal:%.2f(%.2fm), Vel:%.2f]",
    min_vel_x_, max_vel_x_, min_vel_theta_, max_vel_theta_,
    v, w, total_score,
    utility_obs * scale_obs_, obs_raw_cost,
    utility_align * scale_align_, min_dist_to_path,
    utility_goal * scale_heading_, dist_to_goal,
    utility_vel * scale_vel_);
  
  return total_score;
}



} // namespace dwa_controller

PLUGINLIB_EXPORT_CLASS(dwa_controller::DWA_Controller, nav2_core::Controller)