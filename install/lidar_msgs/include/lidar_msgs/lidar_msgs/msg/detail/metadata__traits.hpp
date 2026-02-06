// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__METADATA__TRAITS_HPP_
#define LIDAR_MSGS__MSG__DETAIL__METADATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lidar_msgs/msg/detail/metadata__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace lidar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Metadata & msg,
  std::ostream & out)
{
  out << "{";
  // member: computer_ip
  {
    out << "computer_ip: ";
    rosidl_generator_traits::value_to_yaml(msg.computer_ip, out);
    out << ", ";
  }

  // member: lidar_ip
  {
    out << "lidar_ip: ";
    rosidl_generator_traits::value_to_yaml(msg.lidar_ip, out);
    out << ", ";
  }

  // member: imu_port
  {
    out << "imu_port: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_port, out);
    out << ", ";
  }

  // member: lidar_port
  {
    out << "lidar_port: ";
    rosidl_generator_traits::value_to_yaml(msg.lidar_port, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Metadata & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: computer_ip
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "computer_ip: ";
    rosidl_generator_traits::value_to_yaml(msg.computer_ip, out);
    out << "\n";
  }

  // member: lidar_ip
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lidar_ip: ";
    rosidl_generator_traits::value_to_yaml(msg.lidar_ip, out);
    out << "\n";
  }

  // member: imu_port
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_port: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_port, out);
    out << "\n";
  }

  // member: lidar_port
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lidar_port: ";
    rosidl_generator_traits::value_to_yaml(msg.lidar_port, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Metadata & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace lidar_msgs

namespace rosidl_generator_traits
{

[[deprecated("use lidar_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lidar_msgs::msg::Metadata & msg,
  std::ostream & out, size_t indentation = 0)
{
  lidar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lidar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const lidar_msgs::msg::Metadata & msg)
{
  return lidar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lidar_msgs::msg::Metadata>()
{
  return "lidar_msgs::msg::Metadata";
}

template<>
inline const char * name<lidar_msgs::msg::Metadata>()
{
  return "lidar_msgs/msg/Metadata";
}

template<>
struct has_fixed_size<lidar_msgs::msg::Metadata>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lidar_msgs::msg::Metadata>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lidar_msgs::msg::Metadata>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_MSGS__MSG__DETAIL__METADATA__TRAITS_HPP_
