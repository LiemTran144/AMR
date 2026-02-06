// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__METADATA__BUILDER_HPP_
#define LIDAR_MSGS__MSG__DETAIL__METADATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidar_msgs/msg/detail/metadata__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidar_msgs
{

namespace msg
{

namespace builder
{

class Init_Metadata_lidar_port
{
public:
  explicit Init_Metadata_lidar_port(::lidar_msgs::msg::Metadata & msg)
  : msg_(msg)
  {}
  ::lidar_msgs::msg::Metadata lidar_port(::lidar_msgs::msg::Metadata::_lidar_port_type arg)
  {
    msg_.lidar_port = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_msgs::msg::Metadata msg_;
};

class Init_Metadata_imu_port
{
public:
  explicit Init_Metadata_imu_port(::lidar_msgs::msg::Metadata & msg)
  : msg_(msg)
  {}
  Init_Metadata_lidar_port imu_port(::lidar_msgs::msg::Metadata::_imu_port_type arg)
  {
    msg_.imu_port = std::move(arg);
    return Init_Metadata_lidar_port(msg_);
  }

private:
  ::lidar_msgs::msg::Metadata msg_;
};

class Init_Metadata_lidar_ip
{
public:
  explicit Init_Metadata_lidar_ip(::lidar_msgs::msg::Metadata & msg)
  : msg_(msg)
  {}
  Init_Metadata_imu_port lidar_ip(::lidar_msgs::msg::Metadata::_lidar_ip_type arg)
  {
    msg_.lidar_ip = std::move(arg);
    return Init_Metadata_imu_port(msg_);
  }

private:
  ::lidar_msgs::msg::Metadata msg_;
};

class Init_Metadata_computer_ip
{
public:
  Init_Metadata_computer_ip()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Metadata_lidar_ip computer_ip(::lidar_msgs::msg::Metadata::_computer_ip_type arg)
  {
    msg_.computer_ip = std::move(arg);
    return Init_Metadata_lidar_ip(msg_);
  }

private:
  ::lidar_msgs::msg::Metadata msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_msgs::msg::Metadata>()
{
  return lidar_msgs::msg::builder::Init_Metadata_computer_ip();
}

}  // namespace lidar_msgs

#endif  // LIDAR_MSGS__MSG__DETAIL__METADATA__BUILDER_HPP_
