// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbasic_msgs:srv/Multi.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MSGS__SRV__DETAIL__MULTI__BUILDER_HPP_
#define ROSBASIC_MSGS__SRV__DETAIL__MULTI__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbasic_msgs/srv/detail/multi__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosbasic_msgs
{

namespace srv
{

namespace builder
{

class Init_Multi_Request_b
{
public:
  explicit Init_Multi_Request_b(::rosbasic_msgs::srv::Multi_Request & msg)
  : msg_(msg)
  {}
  ::rosbasic_msgs::srv::Multi_Request b(::rosbasic_msgs::srv::Multi_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbasic_msgs::srv::Multi_Request msg_;
};

class Init_Multi_Request_a
{
public:
  Init_Multi_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Multi_Request_b a(::rosbasic_msgs::srv::Multi_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_Multi_Request_b(msg_);
  }

private:
  ::rosbasic_msgs::srv::Multi_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbasic_msgs::srv::Multi_Request>()
{
  return rosbasic_msgs::srv::builder::Init_Multi_Request_a();
}

}  // namespace rosbasic_msgs


namespace rosbasic_msgs
{

namespace srv
{

namespace builder
{

class Init_Multi_Response_mul
{
public:
  Init_Multi_Response_mul()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbasic_msgs::srv::Multi_Response mul(::rosbasic_msgs::srv::Multi_Response::_mul_type arg)
  {
    msg_.mul = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbasic_msgs::srv::Multi_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbasic_msgs::srv::Multi_Response>()
{
  return rosbasic_msgs::srv::builder::Init_Multi_Response_mul();
}

}  // namespace rosbasic_msgs

#endif  // ROSBASIC_MSGS__SRV__DETAIL__MULTI__BUILDER_HPP_
