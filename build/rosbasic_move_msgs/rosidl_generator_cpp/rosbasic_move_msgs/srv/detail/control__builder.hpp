// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbasic_move_msgs:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__BUILDER_HPP_
#define ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbasic_move_msgs/srv/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosbasic_move_msgs
{

namespace srv
{

namespace builder
{

class Init_Control_Request_khoangcach
{
public:
  explicit Init_Control_Request_khoangcach(::rosbasic_move_msgs::srv::Control_Request & msg)
  : msg_(msg)
  {}
  ::rosbasic_move_msgs::srv::Control_Request khoangcach(::rosbasic_move_msgs::srv::Control_Request::_khoangcach_type arg)
  {
    msg_.khoangcach = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbasic_move_msgs::srv::Control_Request msg_;
};

class Init_Control_Request_quangduong
{
public:
  Init_Control_Request_quangduong()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_Request_khoangcach quangduong(::rosbasic_move_msgs::srv::Control_Request::_quangduong_type arg)
  {
    msg_.quangduong = std::move(arg);
    return Init_Control_Request_khoangcach(msg_);
  }

private:
  ::rosbasic_move_msgs::srv::Control_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbasic_move_msgs::srv::Control_Request>()
{
  return rosbasic_move_msgs::srv::builder::Init_Control_Request_quangduong();
}

}  // namespace rosbasic_move_msgs


namespace rosbasic_move_msgs
{

namespace srv
{

namespace builder
{

class Init_Control_Response_notification
{
public:
  explicit Init_Control_Response_notification(::rosbasic_move_msgs::srv::Control_Response & msg)
  : msg_(msg)
  {}
  ::rosbasic_move_msgs::srv::Control_Response notification(::rosbasic_move_msgs::srv::Control_Response::_notification_type arg)
  {
    msg_.notification = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbasic_move_msgs::srv::Control_Response msg_;
};

class Init_Control_Response_success
{
public:
  Init_Control_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_Response_notification success(::rosbasic_move_msgs::srv::Control_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Control_Response_notification(msg_);
  }

private:
  ::rosbasic_move_msgs::srv::Control_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbasic_move_msgs::srv::Control_Response>()
{
  return rosbasic_move_msgs::srv::builder::Init_Control_Response_success();
}

}  // namespace rosbasic_move_msgs

#endif  // ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__BUILDER_HPP_
