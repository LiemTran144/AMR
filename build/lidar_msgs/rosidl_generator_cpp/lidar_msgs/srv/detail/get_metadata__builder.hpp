// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_msgs:srv/GetMetadata.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__SRV__DETAIL__GET_METADATA__BUILDER_HPP_
#define LIDAR_MSGS__SRV__DETAIL__GET_METADATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidar_msgs/srv/detail/get_metadata__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidar_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_msgs::srv::GetMetadata_Request>()
{
  return ::lidar_msgs::srv::GetMetadata_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace lidar_msgs


namespace lidar_msgs
{

namespace srv
{

namespace builder
{

class Init_GetMetadata_Response_metadata
{
public:
  Init_GetMetadata_Response_metadata()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::lidar_msgs::srv::GetMetadata_Response metadata(::lidar_msgs::srv::GetMetadata_Response::_metadata_type arg)
  {
    msg_.metadata = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_msgs::srv::GetMetadata_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_msgs::srv::GetMetadata_Response>()
{
  return lidar_msgs::srv::builder::Init_GetMetadata_Response_metadata();
}

}  // namespace lidar_msgs

#endif  // LIDAR_MSGS__SRV__DETAIL__GET_METADATA__BUILDER_HPP_
