// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbasic_msgs:srv/AddTwoInts.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MSGS__SRV__DETAIL__ADD_TWO_INTS__BUILDER_HPP_
#define ROSBASIC_MSGS__SRV__DETAIL__ADD_TWO_INTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbasic_msgs/srv/detail/add_two_ints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosbasic_msgs
{

namespace srv
{

namespace builder
{

class Init_AddTwoInts_Request_d
{
public:
  explicit Init_AddTwoInts_Request_d(::rosbasic_msgs::srv::AddTwoInts_Request & msg)
  : msg_(msg)
  {}
  ::rosbasic_msgs::srv::AddTwoInts_Request d(::rosbasic_msgs::srv::AddTwoInts_Request::_d_type arg)
  {
    msg_.d = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbasic_msgs::srv::AddTwoInts_Request msg_;
};

class Init_AddTwoInts_Request_c
{
public:
  Init_AddTwoInts_Request_c()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AddTwoInts_Request_d c(::rosbasic_msgs::srv::AddTwoInts_Request::_c_type arg)
  {
    msg_.c = std::move(arg);
    return Init_AddTwoInts_Request_d(msg_);
  }

private:
  ::rosbasic_msgs::srv::AddTwoInts_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbasic_msgs::srv::AddTwoInts_Request>()
{
  return rosbasic_msgs::srv::builder::Init_AddTwoInts_Request_c();
}

}  // namespace rosbasic_msgs


namespace rosbasic_msgs
{

namespace srv
{

namespace builder
{

class Init_AddTwoInts_Response_sum
{
public:
  Init_AddTwoInts_Response_sum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbasic_msgs::srv::AddTwoInts_Response sum(::rosbasic_msgs::srv::AddTwoInts_Response::_sum_type arg)
  {
    msg_.sum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbasic_msgs::srv::AddTwoInts_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbasic_msgs::srv::AddTwoInts_Response>()
{
  return rosbasic_msgs::srv::builder::Init_AddTwoInts_Response_sum();
}

}  // namespace rosbasic_msgs

#endif  // ROSBASIC_MSGS__SRV__DETAIL__ADD_TWO_INTS__BUILDER_HPP_
