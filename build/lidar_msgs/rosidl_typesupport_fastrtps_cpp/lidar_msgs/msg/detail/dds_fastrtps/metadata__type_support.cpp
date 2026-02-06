// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from lidar_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice
#include "lidar_msgs/msg/detail/metadata__rosidl_typesupport_fastrtps_cpp.hpp"
#include "lidar_msgs/msg/detail/metadata__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace lidar_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lidar_msgs
cdr_serialize(
  const lidar_msgs::msg::Metadata & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: computer_ip
  cdr << ros_message.computer_ip;
  // Member: lidar_ip
  cdr << ros_message.lidar_ip;
  // Member: imu_port
  cdr << ros_message.imu_port;
  // Member: lidar_port
  cdr << ros_message.lidar_port;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lidar_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  lidar_msgs::msg::Metadata & ros_message)
{
  // Member: computer_ip
  cdr >> ros_message.computer_ip;

  // Member: lidar_ip
  cdr >> ros_message.lidar_ip;

  // Member: imu_port
  cdr >> ros_message.imu_port;

  // Member: lidar_port
  cdr >> ros_message.lidar_port;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lidar_msgs
get_serialized_size(
  const lidar_msgs::msg::Metadata & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: computer_ip
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.computer_ip.size() + 1);
  // Member: lidar_ip
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.lidar_ip.size() + 1);
  // Member: imu_port
  {
    size_t item_size = sizeof(ros_message.imu_port);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: lidar_port
  {
    size_t item_size = sizeof(ros_message.lidar_port);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lidar_msgs
max_serialized_size_Metadata(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: computer_ip
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: lidar_ip
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: imu_port
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: lidar_port
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = lidar_msgs::msg::Metadata;
    is_plain =
      (
      offsetof(DataType, lidar_port) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Metadata__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const lidar_msgs::msg::Metadata *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Metadata__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<lidar_msgs::msg::Metadata *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Metadata__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const lidar_msgs::msg::Metadata *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Metadata__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Metadata(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Metadata__callbacks = {
  "lidar_msgs::msg",
  "Metadata",
  _Metadata__cdr_serialize,
  _Metadata__cdr_deserialize,
  _Metadata__get_serialized_size,
  _Metadata__max_serialized_size
};

static rosidl_message_type_support_t _Metadata__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Metadata__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace lidar_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_lidar_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<lidar_msgs::msg::Metadata>()
{
  return &lidar_msgs::msg::typesupport_fastrtps_cpp::_Metadata__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, lidar_msgs, msg, Metadata)() {
  return &lidar_msgs::msg::typesupport_fastrtps_cpp::_Metadata__handle;
}

#ifdef __cplusplus
}
#endif
