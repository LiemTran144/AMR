// #include "liem_controller/path_follower.hpp"


// namespace liem_controller
// {

// PathFollower::PathFollower() : Node("path_follower_node")
// {

//   client_ = rclcpp_action::create_client<FollowPath>(
//         this, "/follow_path");

//   sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
//     "/plan",
//     10,
//     std::bind(&PathFollower::path_callback, this, std::placeholders::_1));

//   RCLCPP_INFO(this->get_logger(), "Path Follower Node (C++) initialized. Waiting for /plan...");
// }

// void PathFollower::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
// {
//   RCLCPP_INFO(this->get_logger(), "Received path with %zu poses.", msg->poses.size());

//   if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
//     RCLCPP_ERROR(this->get_logger(), "Action server 'follow_path' not available.");
//     return;
//   }

//   auto goal_msg = FollowPath::Goal();
//   goal_msg.path = *msg;
//   goal_msg.controller_id = "FollowPath";
//   goal_msg.goal_checker_id = "general_goal_checker";

//   auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  
//   send_goal_options.goal_response_callback =
//     std::bind(&PathFollower::goal_response_callback, this, std::placeholders::_1);
    
//   send_goal_options.feedback_callback =
//     std::bind(&PathFollower::feedback_callback, this, _1, _2);
    
//   send_goal_options.result_callback =
//     std::bind(&PathFollower::result_callback, this, _1);

//   RCLCPP_INFO(this->get_logger(), "Sending goal to controller...");
//   this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
// }

// void PathFollower::goal_response_callback(const GoalHandleFollowPath::SharedPtr & goal_handle)
// {
//   if (!goal_handle) {
//     RCLCPP_ERROR(this->get_logger(), "Goal was rejected by controller");
//   } else {
//     RCLCPP_INFO(this->get_logger(), "Goal accepted by controller, executing...");
//   }
// }

// void PathFollower::feedback_callback(
//   GoalHandleFollowPath::SharedPtr,
//   const std::shared_ptr<const FollowPath::Feedback> feedback)
// {
//   // In ra log ít thôi để đỡ rối màn hình (throttle log)
//   RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
//     "Speed: %.2f m/s, Dist to goal: %.2f m", 
//     feedback->speed, feedback->distance_to_goal);
// }

// void PathFollower::result_callback(const GoalHandleFollowPath::WrappedResult & result)
// {
//   switch (result.code) {
//     case rclcpp_action::ResultCode::SUCCEEDED:
//       RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
//       break;
//     case rclcpp_action::ResultCode::ABORTED:
//       RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//       break;
//     case rclcpp_action::ResultCode::CANCELED:
//       RCLCPP_WARN(this->get_logger(), "Goal was canceled");
//       break;
//     default:
//       RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//       break;
//   }
// }

// }  // namespace liem_controller

// // --- Main Function ---
// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<liem_controller::PathFollower>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }







#include "liem_controller/path_follower_client.hpp"
#include <chrono>
#include <functional> // Quan trọng để dùng std::bind

namespace liem_controller
{

PathFollower::PathFollower(const rclcpp::NodeOptions & options): Node("path_follower_node", options)
{
    // create action client to /follow_path
    client_ = rclcpp_action::create_client<FollowPath>(
        this, "follow_path");

    // subscribe path from /plan (output cua A*)
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10,
        std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));
}

void PathFollower::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // wait for action server
    if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available");
        return;
    }

    auto goal = FollowPath::Goal();
    goal.path = *msg;
    goal.controller_id = "FollowPath";
    goal.goal_checker_id = "general_goal_checker";

    RCLCPP_INFO(this->get_logger(), "Received Path with %zu poses. Sending to Controller...", msg->poses.size());

    // Cấu hình callback để nhận phản hồi (feedback/result)
    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        std::bind(&PathFollower::goal_response_callback, this, std::placeholders::_1);
    
    send_goal_options.feedback_callback =
        std::bind(&PathFollower::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback =
        std::bind(&PathFollower::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal, send_goal_options);
}

// --- Các hàm callback bổ trợ (giữ nguyên logic để debug) ---

void PathFollower::goal_response_callback(const GoalHandleFollowPath::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by controller");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by controller");
    }
}

void PathFollower::feedback_callback(GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback)
{
    // Throttle log để không bị spam màn hình
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Speed: %.2f m/s, Dist to goal: %.2f m", 
        feedback->speed, feedback->distance_to_goal);
}

void PathFollower::result_callback(const GoalHandleFollowPath::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}

} // namespace liem_controller

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<liem_controller::PathFollower>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}