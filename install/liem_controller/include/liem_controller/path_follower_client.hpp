// #ifndef LIEM_CONTROLLER__PATH_FOLLOWER_HPP_
// #define LIEM_CONTROLLER__PATH_FOLLOWER_HPP_

// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "nav2_msgs/action/follow_path.hpp"
// #include "nav_msgs/msg/path.hpp"

// namespace liem_controller
// {

// class PathFollower : public rclcpp::Node
// {
// public:
//   // Định nghĩa các kiểu dữ liệu cho ngắn gọn
//   using FollowPath = nav2_msgs::action::FollowPath;
//   using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

//   // Constructor
//   explicit PathFollower();
  
//   virtual ~PathFollower() = default;

// private:
//   // --- Các biến thành viên (Variables) ---
//   rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
//   rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;

//   // --- Các hàm Callback (Methods) ---
  
//   // Xử lý khi nhận được Path từ A*
//   void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

//   // Xử lý phản hồi từ Action Server (Chấp nhận/Từ chối)
//   void goal_response_callback(const GoalHandleFollowPath::SharedPtr & goal_handle);

//   // Xử lý Feedback liên tục (Vận tốc, khoảng cách)
//   void feedback_callback(
//     GoalHandleFollowPath::SharedPtr,
//     const std::shared_ptr<const FollowPath::Feedback> feedback);

//   // Xử lý kết quả cuối cùng (Thành công/Thất bại)
//   void result_callback(const GoalHandleFollowPath::WrappedResult & result);
// };

// }  // namespace liem_controller

// #endif  // LIEM_CONTROLLER__PATH_FOLLOWER_HPP_




#ifndef LIEM_CONTROLLER__PATH_FOLLOWER_HPP_
#define LIEM_CONTROLLER__PATH_FOLLOWER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"

namespace liem_controller
{

class PathFollower : public rclcpp::Node
{
public:
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    // Cập nhật Constructor theo format mới
    explicit PathFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~PathFollower() = default;

private:
    rclcpp_action::Client<FollowPath>::SharedPtr client_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // Các hàm xử lý kết quả action
    void goal_response_callback(const GoalHandleFollowPath::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback);
    void result_callback(const GoalHandleFollowPath::WrappedResult & result);
};

}  // namespace liem_controller

#endif  // LIEM_CONTROLLER__PATH_FOLLOWER_HPP_