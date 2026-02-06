// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosbasic_move_msgs:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__TRAITS_HPP_
#define ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosbasic_move_msgs/srv/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosbasic_move_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Control_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: quangduong
  {
    out << "quangduong: ";
    rosidl_generator_traits::value_to_yaml(msg.quangduong, out);
    out << ", ";
  }

  // member: khoangcach
  {
    out << "khoangcach: ";
    rosidl_generator_traits::value_to_yaml(msg.khoangcach, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Control_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: quangduong
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quangduong: ";
    rosidl_generator_traits::value_to_yaml(msg.quangduong, out);
    out << "\n";
  }

  // member: khoangcach
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "khoangcach: ";
    rosidl_generator_traits::value_to_yaml(msg.khoangcach, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Control_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rosbasic_move_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rosbasic_move_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosbasic_move_msgs::srv::Control_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbasic_move_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbasic_move_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbasic_move_msgs::srv::Control_Request & msg)
{
  return rosbasic_move_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbasic_move_msgs::srv::Control_Request>()
{
  return "rosbasic_move_msgs::srv::Control_Request";
}

template<>
inline const char * name<rosbasic_move_msgs::srv::Control_Request>()
{
  return "rosbasic_move_msgs/srv/Control_Request";
}

template<>
struct has_fixed_size<rosbasic_move_msgs::srv::Control_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosbasic_move_msgs::srv::Control_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosbasic_move_msgs::srv::Control_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosbasic_move_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const Control_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: notification
  {
    out << "notification: ";
    rosidl_generator_traits::value_to_yaml(msg.notification, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Control_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: notification
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "notification: ";
    rosidl_generator_traits::value_to_yaml(msg.notification, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Control_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace rosbasic_move_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rosbasic_move_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosbasic_move_msgs::srv::Control_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbasic_move_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbasic_move_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbasic_move_msgs::srv::Control_Response & msg)
{
  return rosbasic_move_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbasic_move_msgs::srv::Control_Response>()
{
  return "rosbasic_move_msgs::srv::Control_Response";
}

template<>
inline const char * name<rosbasic_move_msgs::srv::Control_Response>()
{
  return "rosbasic_move_msgs/srv/Control_Response";
}

template<>
struct has_fixed_size<rosbasic_move_msgs::srv::Control_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosbasic_move_msgs::srv::Control_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosbasic_move_msgs::srv::Control_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rosbasic_move_msgs::srv::Control>()
{
  return "rosbasic_move_msgs::srv::Control";
}

template<>
inline const char * name<rosbasic_move_msgs::srv::Control>()
{
  return "rosbasic_move_msgs/srv/Control";
}

template<>
struct has_fixed_size<rosbasic_move_msgs::srv::Control>
  : std::integral_constant<
    bool,
    has_fixed_size<rosbasic_move_msgs::srv::Control_Request>::value &&
    has_fixed_size<rosbasic_move_msgs::srv::Control_Response>::value
  >
{
};

template<>
struct has_bounded_size<rosbasic_move_msgs::srv::Control>
  : std::integral_constant<
    bool,
    has_bounded_size<rosbasic_move_msgs::srv::Control_Request>::value &&
    has_bounded_size<rosbasic_move_msgs::srv::Control_Response>::value
  >
{
};

template<>
struct is_service<rosbasic_move_msgs::srv::Control>
  : std::true_type
{
};

template<>
struct is_service_request<rosbasic_move_msgs::srv::Control_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rosbasic_move_msgs::srv::Control_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__TRAITS_HPP_
