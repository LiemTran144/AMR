// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rosbasic_msgs:srv/AddTwoInts.idl
// generated code does not contain a copyright notice
#include "rosbasic_msgs/srv/detail/add_two_ints__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosbasic_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rosbasic_msgs/srv/detail/add_two_ints__struct.h"
#include "rosbasic_msgs/srv/detail/add_two_ints__functions.h"
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


// forward declare type support functions


using _AddTwoInts_Request__ros_msg_type = rosbasic_msgs__srv__AddTwoInts_Request;

static bool _AddTwoInts_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _AddTwoInts_Request__ros_msg_type * ros_message = static_cast<const _AddTwoInts_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: c
  {
    cdr << ros_message->c;
  }

  // Field name: d
  {
    cdr << ros_message->d;
  }

  return true;
}

static bool _AddTwoInts_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _AddTwoInts_Request__ros_msg_type * ros_message = static_cast<_AddTwoInts_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: c
  {
    cdr >> ros_message->c;
  }

  // Field name: d
  {
    cdr >> ros_message->d;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosbasic_msgs
size_t get_serialized_size_rosbasic_msgs__srv__AddTwoInts_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AddTwoInts_Request__ros_msg_type * ros_message = static_cast<const _AddTwoInts_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name c
  {
    size_t item_size = sizeof(ros_message->c);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name d
  {
    size_t item_size = sizeof(ros_message->d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _AddTwoInts_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rosbasic_msgs__srv__AddTwoInts_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosbasic_msgs
size_t max_serialized_size_rosbasic_msgs__srv__AddTwoInts_Request(
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

  // member: c
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rosbasic_msgs__srv__AddTwoInts_Request;
    is_plain =
      (
      offsetof(DataType, d) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _AddTwoInts_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rosbasic_msgs__srv__AddTwoInts_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AddTwoInts_Request = {
  "rosbasic_msgs::srv",
  "AddTwoInts_Request",
  _AddTwoInts_Request__cdr_serialize,
  _AddTwoInts_Request__cdr_deserialize,
  _AddTwoInts_Request__get_serialized_size,
  _AddTwoInts_Request__max_serialized_size
};

static rosidl_message_type_support_t _AddTwoInts_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AddTwoInts_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rosbasic_msgs, srv, AddTwoInts_Request)() {
  return &_AddTwoInts_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosbasic_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "rosbasic_msgs/srv/detail/add_two_ints__struct.h"
// already included above
// #include "rosbasic_msgs/srv/detail/add_two_ints__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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


// forward declare type support functions


using _AddTwoInts_Response__ros_msg_type = rosbasic_msgs__srv__AddTwoInts_Response;

static bool _AddTwoInts_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _AddTwoInts_Response__ros_msg_type * ros_message = static_cast<const _AddTwoInts_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: sum
  {
    cdr << ros_message->sum;
  }

  return true;
}

static bool _AddTwoInts_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _AddTwoInts_Response__ros_msg_type * ros_message = static_cast<_AddTwoInts_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: sum
  {
    cdr >> ros_message->sum;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosbasic_msgs
size_t get_serialized_size_rosbasic_msgs__srv__AddTwoInts_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AddTwoInts_Response__ros_msg_type * ros_message = static_cast<const _AddTwoInts_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name sum
  {
    size_t item_size = sizeof(ros_message->sum);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _AddTwoInts_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rosbasic_msgs__srv__AddTwoInts_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosbasic_msgs
size_t max_serialized_size_rosbasic_msgs__srv__AddTwoInts_Response(
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

  // member: sum
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rosbasic_msgs__srv__AddTwoInts_Response;
    is_plain =
      (
      offsetof(DataType, sum) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _AddTwoInts_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rosbasic_msgs__srv__AddTwoInts_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AddTwoInts_Response = {
  "rosbasic_msgs::srv",
  "AddTwoInts_Response",
  _AddTwoInts_Response__cdr_serialize,
  _AddTwoInts_Response__cdr_deserialize,
  _AddTwoInts_Response__get_serialized_size,
  _AddTwoInts_Response__max_serialized_size
};

static rosidl_message_type_support_t _AddTwoInts_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AddTwoInts_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rosbasic_msgs, srv, AddTwoInts_Response)() {
  return &_AddTwoInts_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosbasic_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rosbasic_msgs/srv/add_two_ints.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t AddTwoInts__callbacks = {
  "rosbasic_msgs::srv",
  "AddTwoInts",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rosbasic_msgs, srv, AddTwoInts_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rosbasic_msgs, srv, AddTwoInts_Response)(),
};

static rosidl_service_type_support_t AddTwoInts__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &AddTwoInts__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rosbasic_msgs, srv, AddTwoInts)() {
  return &AddTwoInts__handle;
}

#if defined(__cplusplus)
}
#endif
