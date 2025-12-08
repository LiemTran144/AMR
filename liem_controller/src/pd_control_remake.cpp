#include "liem_controller/pd_control_remake.hpp"

namespace liem_controller
{

PDControl::PDControl() : Node("pd_motion_planner_node"),
  current_state_(STATE_IDLE),
  prev_angular_error_(0.0),
  prev_linear_error_(0.0),
  final_goal_yaw_(0.0),
  has_plan_(false)
{
    // --- Parameters ---
    this->declare_parameter("kp_linear", 1.5);
    this->declare_parameter("kd_linear", 0.01);
    this->declare_parameter("kp_angular", 1.5);
    this->declare_parameter("kd_angular", 0.01);
    
    this->declare_parameter("step_size", 0.3);
    this->declare_parameter("max_linear_velocity", 0.2);
    this->declare_parameter("max_angular_velocity", 0.3);
    
    this->declare_parameter("xy_goal_tolerance", 0.10);
    this->declare_parameter("yaw_goal_tolerance", 0.15);

    kp_lin_ = this->get_parameter("kp_linear").as_double();
    kd_lin_ = this->get_parameter("kd_linear").as_double();
    kp_ang_ = this->get_parameter("kp_angular").as_double();
    kd_ang_ = this->get_parameter("kd_angular").as_double();

    step_size_ = this->get_parameter("step_size").as_double();
    max_v_ = this->get_parameter("max_linear_velocity").as_double();
    max_w_ = this->get_parameter("max_angular_velocity").as_double();

    xy_tol_ = this->get_parameter("xy_goal_tolerance").as_double();
    yaw_tol_ = this->get_parameter("yaw_goal_tolerance").as_double();

    // --- TF Setup ---
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // --- Subs/Pubs ---
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&PDControl::pathCallback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PDControl::goalCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/nav/cmd_vel", 10);
    next_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);

    // --- Timer ---
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PDControl::controlLoop, this));
    
    last_cycle_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "PD Control Remake Initialized");
}

void PDControl::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    RCLCPP_INFO(this->get_logger(), "New path received");
    global_plan_ = *path;
    has_plan_ = true;
    current_state_ = STATE_ALIGN_START;
    prev_angular_error_ = 0.0;
    prev_linear_error_ = 0.0;
    last_cycle_time_ = this->now();
}

void PDControl::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    try {
        // 1. Get Transform Goal Frame (map) -> Odom
        geometry_msgs::msg::TransformStamped trans_msg;
        trans_msg = tf_buffer_->lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);

        // 2. Convert to TF2 objects for matrix multiplication
        tf2::Transform map_to_odom_tf;
        tf2::fromMsg(trans_msg.transform, map_to_odom_tf);

        tf2::Transform goal_pose_tf;
        tf2::fromMsg(msg->pose, goal_pose_tf);

        // 3. Multiply: Pose_Odom = Transform * Pose_Map
        tf2::Transform final_tf = map_to_odom_tf * goal_pose_tf;

        // 4. Extract Yaw
        double roll, pitch, yaw;
        final_tf.getBasis().getRPY(roll, pitch, yaw);
        
        final_goal_yaw_ = yaw;
        RCLCPP_INFO(this->get_logger(), "Goal converted to Odom Yaw: %.3f rad", final_goal_yaw_);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform goal pose: %s", ex.what());
    }
}

void PDControl::controlLoop()
{
    if (!has_plan_ || global_plan_.poses.empty()) {
        return;
    }

    // 1. Get Robot Pose in Odom
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        
        robot_pose.header = tf_stamped.header;
        robot_pose.pose.position.x = tf_stamped.transform.translation.x;
        robot_pose.pose.position.y = tf_stamped.transform.translation.y;
        robot_pose.pose.position.z = tf_stamped.transform.translation.z;
        robot_pose.pose.orientation = tf_stamped.transform.rotation;
        
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
        return;
    }

    // 2. Transform Plan to Robot Frame (Odom)
    // Note: In C++, we can modify member variable global_plan_ directly if we want persistence
    if (!transformPlan(robot_pose.header.frame_id)) {
        RCLCPP_ERROR(this->get_logger(), "Unable to transform Plan");
        return;
    }

    // 3. Find Next Pose (Lookahead)
    geometry_msgs::msg::PoseStamped next_pose = getNextPose(robot_pose);
      
    // Calculate distance to Final Goal
    double dx_final = global_plan_.poses.back().pose.position.x - robot_pose.pose.position.x;
    double dy_final = global_plan_.poses.back().pose.position.y - robot_pose.pose.position.y;
    double dist_to_goal = std::hypot(dx_final, dy_final);

    next_pose_pub_->publish(next_pose);

    tf2::Transform odom_to_robot_tf;
    tf2::fromMsg(robot_pose.pose, odom_to_robot_tf);
    
    tf2::Transform odom_to_target_tf;
    tf2::fromMsg(next_pose.pose, odom_to_target_tf);
    
    // robot_inv * target = local_target
    tf2::Transform robot_to_target_tf = odom_to_robot_tf.inverse() * odom_to_target_tf;
    
    double local_x = robot_to_target_tf.getOrigin().x();
    double local_y = robot_to_target_tf.getOrigin().y();

    double heading_error = std::atan2(local_y, local_x);   
    double linear_error = std::hypot(local_x, local_y);

    // 5. Time Calculation
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_cycle_time_).seconds();
    if (dt <= 0.0) dt = 0.1;

    geometry_msgs::msg::Twist cmd_vel;

    // ============= STATE MACHINE =============

    if (current_state_ == STATE_ALIGN_START) {
        RCLCPP_INFO(this->get_logger(), "Start Aligning!");
        if (std::abs(heading_error) < yaw_tol_) {
            RCLCPP_INFO(this->get_logger(), "Start Aligned! -> Switch to TRACKING");
            current_state_ = STATE_TRACKING;
            // current_state_ = 20; 
            prev_angular_error_ = 0.0;
        } else {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = calcPidAngular(local_y, dt);
            // cmd_vel.angular.z = calcPidAngular(heading_error, dt);
        }
    }
    else if (current_state_ == STATE_TRACKING) {
        if (dist_to_goal < xy_tol_) {
            RCLCPP_INFO(this->get_logger(), "XY Goal Reached! -> Switch to ALIGN_GOAL");
            current_state_ = STATE_ALIGN_GOAL;
        } else {
            // Tracking
            RCLCPP_INFO(this->get_logger(), "Tracking... X: %.3f, Y: %.3f", local_x, local_y);
            cmd_vel.linear.x = calcPidLinear(local_x, dt); 
            cmd_vel.angular.z = calcPidAngular(local_y, dt); 
            // cmd_vel.linear.x = calcPidLinear(linear_reror, dt);
            // cmd_vel.angular.z = calcPidAngular(heading_error, dt);
        }
    }
    else if (current_state_ == STATE_ALIGN_GOAL) {
        double current_yaw = getYaw(robot_pose.pose);
        double delta = final_goal_yaw_ - current_yaw;
        
        // Normalize angle
        double yaw_error = std::atan2(std::sin(delta), std::cos(delta));

        if (std::abs(yaw_error) < yaw_tol_) {
            RCLCPP_INFO(this->get_logger(), "COMPLETE! Goal Reached. Final Pos: %.3f, %.3f", 
                robot_pose.pose.position.x, robot_pose.pose.position.y);
            RCLCPP_INFO(this->get_logger(), "Final Error: %.3f m, %.3f rad",dist_to_goal, yaw_error);
            has_plan_ = false; // Stop loop
            current_state_ = STATE_IDLE;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        } else {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = calcPidAngular(yaw_error, dt);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Cmd Vel: Linear %.3f m/s, Angular %.3f rad/s", 
        cmd_vel.linear.x, cmd_vel.angular.z);
    cmd_pub_->publish(cmd_vel);
    last_cycle_time_ = current_time;
}

bool PDControl::transformPlan(const std::string& target_frame)
{
    if (global_plan_.header.frame_id == target_frame) {
        return true;
    }

    try {
        geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
            target_frame, global_plan_.header.frame_id, tf2::TimePointZero);

        // We use doTransform from tf2_geometry_msgs to transform the whole path efficiently
        for (auto & pose_stamped : global_plan_.poses) {
            tf2::doTransform(pose_stamped, pose_stamped, tf_stamped);
        }
        global_plan_.header.frame_id = target_frame;
        return true;

    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "TF Error in transformPlan: %s", ex.what());
        return false;
    }
}

geometry_msgs::msg::PoseStamped PDControl::getNextPose(const geometry_msgs::msg::PoseStamped& robot_pose)
{
    geometry_msgs::msg::PoseStamped next_pose = global_plan_.poses.back();
    
    // Iterate backwards
    for (int i = global_plan_.poses.size() - 1; i >= 0; --i) {
        const auto& pose = global_plan_.poses[i];
        double dx = pose.pose.position.x - robot_pose.pose.position.x;
        double dy = pose.pose.position.y - robot_pose.pose.position.y;
        double dist = std::hypot(dx, dy);
        
        if (dist > step_size_) {
            next_pose = pose;
        } else {
            break; 
        }
    }
    return next_pose;
}

double PDControl::getYaw(const geometry_msgs::msg::Pose& pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

double PDControl::calcPidAngular(double error, double dt)
{
    double derivative = (error - prev_angular_error_) / dt;
    double output = kp_ang_ * error + kd_ang_ * derivative;
    prev_angular_error_ = error;
    
    // Clamp
    return std::max(-max_w_, std::min(output, max_w_));
}

double PDControl::calcPidLinear(double error, double dt)
{
    double derivative = (error - prev_linear_error_) / dt;
    double output = kp_lin_ * error + kd_lin_ * derivative;
    prev_linear_error_ = error;
    
    // Clamp
    return std::max(0.0, std::min(output, max_v_));
}

} // namespace liem_controller

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<liem_controller::PDControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}