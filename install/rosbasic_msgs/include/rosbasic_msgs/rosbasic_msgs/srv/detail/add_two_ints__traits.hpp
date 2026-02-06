// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosbasic_msgs:srv/AddTwoInts.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MSGS__SRV__DETAIL__ADD_TWO_INTS__TRAITS_HPP_
#define ROSBASIC_MSGS__SRV__DETAIL__ADD_TWO_INTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosbasic_msgs/srv/detail/add_two_ints__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosbasic_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const AddTwoInts_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: c
  {
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << ", ";
  }

  // member: d
  {
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AddTwoInts_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: c
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "c: ";
    rosidl_generator_traits::value_to_yaml(msg.c, out);
    out << "\n";
  }

  // member: d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d: ";
    rosidl_generator_traits::value_to_yaml(msg.d, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AddTwoInts_Request & msg, bool use_flow_style = false)
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

}  // namespace rosbasic_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rosbasic_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosbasic_msgs::srv::AddTwoInts_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbasic_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbasic_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbasic_msgs::srv::AddTwoInts_Request & msg)
{
  return rosbasic_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbasic_msgs::srv::AddTwoInts_Request>()
{
  return "rosbasic_msgs::srv::AddTwoInts_Request";
}

template<>
inline const char * name<rosbasic_msgs::srv::AddTwoInts_Request>()
{
  return "rosbasic_msgs/srv/AddTwoInts_Request";
}

template<>
struct has_fixed_size<rosbasic_msgs::srv::AddTwoInts_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosbasic_msgs::srv::AddTwoInts_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosbasic_msgs::srv::AddTwoInts_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosbasic_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const AddTwoInts_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: sum
  {
    out << "sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sum, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AddTwoInts_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sum: ";
    rosidl_generator_traits::value_to_yaml(msg.sum, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AddTwoInts_Response & msg, bool use_flow_style = false)
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

}  // namespace rosbasic_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rosbasic_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosbasic_msgs::srv::AddTwoInts_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosbasic_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosbasic_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const rosbasic_msgs::srv::AddTwoInts_Response & msg)
{
  return rosbasic_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rosbasic_msgs::srv::AddTwoInts_Response>()
{
  return "rosbasic_msgs::srv::AddTwoInts_Response";
}

template<>
inline const char * name<rosbasic_msgs::srv::AddTwoInts_Response>()
{
  return "rosbasic_msgs/srv/AddTwoInts_Response";
}

template<>
struct has_fixed_size<rosbasic_msgs::srv::AddTwoInts_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rosbasic_msgs::srv::AddTwoInts_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rosbasic_msgs::srv::AddTwoInts_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rosbasic_msgs::srv::AddTwoInts>()
{
  return "rosbasic_msgs::srv::AddTwoInts";
}

template<>
inline const char * name<rosbasic_msgs::srv::AddTwoInts>()
{
  return "rosbasic_msgs/srv/AddTwoInts";
}

template<>
struct has_fixed_size<rosbasic_msgs::srv::AddTwoInts>
  : std::integral_constant<
    bool,
    has_fixed_size<rosbasic_msgs::srv::AddTwoInts_Request>::value &&
    has_fixed_size<rosbasic_msgs::srv::AddTwoInts_Response>::value
  >
{
};

template<>
struct has_bounded_size<rosbasic_msgs::srv::AddTwoInts>
  : std::integral_constant<
    bool,
    has_bounded_size<rosbasic_msgs::srv::AddTwoInts_Request>::value &&
    has_bounded_size<rosbasic_msgs::srv::AddTwoInts_Response>::value
  >
{
};

template<>
struct is_service<rosbasic_msgs::srv::AddTwoInts>
  : std::true_type
{
};

template<>
struct is_service_request<rosbasic_msgs::srv::AddTwoInts_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rosbasic_msgs::srv::AddTwoInts_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROSBASIC_MSGS__SRV__DETAIL__ADD_TWO_INTS__TRAITS_HPP_
