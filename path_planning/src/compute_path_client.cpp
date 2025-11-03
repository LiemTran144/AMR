#include "path_planning/compute_path_client.hpp"

#include <chrono>
#include <utility>

namespace compute_path_client
{

ComputePathClient::ComputePathClient(const rclcpp::NodeOptions & options)
: Node("compute_path_client", options)
{
    // create action client to /compute_path_to_pose
    client_ = rclcpp_action::create_client<ComputePathToPose>(
        this, "/compute_path_to_pose");

    // subscribe goal from /goal_pose
    sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&ComputePathClient::goalCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ComputePathClient started");
}

void ComputePathClient::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // wait for action server
    if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "ComputePathToPose action server not available");
        return;
    }

    auto goal = ComputePathToPose::Goal();
    goal.goal = *msg;
    goal.planner_id = "GridBased"; 

    RCLCPP_INFO(this->get_logger(),"Sending goal to compute_path_to_pose: (%.2f, %.2f)",msg->pose.position.x, msg->pose.position.y);

    client_->async_send_goal(goal);
}

} 

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<compute_path_client::ComputePathClient>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}