// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from lidar_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice
#include "lidar_msgs/msg/detail/metadata__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "lidar_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "lidar_msgs/msg/detail/metadata__struct.h"
#include "lidar_msgs/msg/detail/metadata__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // computer_ip, lidar_ip
#include "rosidl_runtime_c/string_functions.h"  // computer_ip, lidar_ip

// forward declare type support functions


using _Metadata__ros_msg_type = lidar_msgs__msg__Metadata;

static bool _Metadata__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Metadata__ros_msg_type * ros_message = static_cast<const _Metadata__ros_msg_type *>(untyped_ros_message);
  // Field name: computer_ip
  {
    const rosidl_runtime_c__String * str = &ros_message->computer_ip;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: lidar_ip
  {
    const rosidl_runtime_c__String * str = &ros_message->lidar_ip;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: imu_port
  {
    cdr << ros_message->imu_port;
  }

  // Field name: lidar_port
  {
    cdr << ros_message->lidar_port;
  }

  return true;
}

static bool _Metadata__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Metadata__ros_msg_type * ros_message = static_cast<_Metadata__ros_msg_type *>(untyped_ros_message);
  // Field name: computer_ip
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->computer_ip.data) {
      rosidl_runtime_c__String__init(&ros_message->computer_ip);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->computer_ip,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'computer_ip'\n");
      return false;
    }
  }

  // Field name: lidar_ip
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->lidar_ip.data) {
      rosidl_runtime_c__String__init(&ros_message->lidar_ip);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->lidar_ip,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'lidar_ip'\n");
      return false;
    }
  }

  // Field name: imu_port
  {
    cdr >> ros_message->imu_port;
  }

  // Field name: lidar_port
  {
    cdr >> ros_message->lidar_port;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_msgs
size_t get_serialized_size_lidar_msgs__msg__Metadata(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Metadata__ros_msg_type * ros_message = static_cast<const _Metadata__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name computer_ip
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->computer_ip.size + 1);
  // field.name lidar_ip
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->lidar_ip.size + 1);
  // field.name imu_port
  {
    size_t item_size = sizeof(ros_message->imu_port);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lidar_port
  {
    size_t item_size = sizeof(ros_message->lidar_port);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Metadata__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_lidar_msgs__msg__Metadata(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_msgs
size_t max_serialized_size_lidar_msgs__msg__Metadata(
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

  // member: computer_ip
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
  // member: lidar_ip
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
  // member: imu_port
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: lidar_port
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
    using DataType = lidar_msgs__msg__Metadata;
    is_plain =
      (
      offsetof(DataType, lidar_port) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Metadata__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_lidar_msgs__msg__Metadata(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Metadata = {
  "lidar_msgs::msg",
  "Metadata",
  _Metadata__cdr_serialize,
  _Metadata__cdr_deserialize,
  _Metadata__get_serialized_size,
  _Metadata__max_serialized_size
};

static rosidl_message_type_support_t _Metadata__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Metadata,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lidar_msgs, msg, Metadata)() {
  return &_Metadata__type_support;
}

#if defined(__cplusplus)
}
#endif
