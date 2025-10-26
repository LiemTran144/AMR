#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleTalker(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("lifecycle_talker", options)
  {
    RCLCPP_INFO(get_logger(), "LifecycleTalker constructor called.");
  }

  // Called when transitioning from "unconfigured" -> "inactive"
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_configure() called.");

    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "demo_chatter", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
      1s, std::bind(&LifecycleTalker::on_timer, this));

    // timer disabled by default (only active when node activated)
    timer_->cancel();

    msg_count_ = 0;
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Called when transitioning from "inactive" -> "active"
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_activate() called.");
    publisher_->on_activate();
    timer_->reset();  // start timer
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Called when transitioning from "active" -> "inactive"
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_deactivate() called.");
    timer_->cancel();
    publisher_->on_deactivate();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Called when transitioning from any state -> "finalized"
LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
{
  if (timer_ && timer_->is_canceled()) {
      timer_->cancel();
  }
  RCLCPP_INFO(get_logger(), "on_shutdown() called.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


private:
  void on_timer()
  {
    if (!publisher_->is_activated())
      return;

    auto msg = std_msgs::msg::String();
    msg.data = "Hello from LifecycleTalker! Count = " + std::to_string(msg_count_++);
    RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int msg_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleTalker>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
