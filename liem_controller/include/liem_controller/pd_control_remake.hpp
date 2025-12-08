#ifndef PD_CONTROL_HPP_
#define PD_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

namespace liem_controller
{

enum State {
    STATE_IDLE = 0,
    STATE_ALIGN_START = 1,
    STATE_TRACKING = 2,
    STATE_ALIGN_GOAL = 3
};

class PDControl : public rclcpp::Node
{
public:
    PDControl();
    ~PDControl() = default;

private:
    // --- Callbacks ---
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlLoop();

    // --- Helper Functions ---
    bool transformPlan(const std::string& target_frame);
    geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped& robot_pose);
    double getYaw(const geometry_msgs::msg::Pose& pose);
    double normalizeAngle(double angle);

    double calcPidAngular(double error, double dt);
    double calcPidLinear(double error, double dt);

    // --- ROS Interfaces ---
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // --- Variables ---
    nav_msgs::msg::Path global_plan_;
    int current_state_;
    
    double prev_angular_error_;
    double prev_linear_error_;
    rclcpp::Time last_cycle_time_;
    
    double final_goal_yaw_;
    bool has_plan_;

    // --- Parameters ---
    double kp_lin_, kd_lin_;
    double kp_ang_, kd_ang_;
    double step_size_;
    double max_v_, max_w_;
    double xy_tol_, yaw_tol_;
};

} // namespace liem_controller

#endif // PD_CONTROL_HPP_