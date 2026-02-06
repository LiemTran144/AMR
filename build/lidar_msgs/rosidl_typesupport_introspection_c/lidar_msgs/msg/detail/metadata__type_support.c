// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidar_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidar_msgs/msg/detail/metadata__rosidl_typesupport_introspection_c.h"
#include "lidar_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidar_msgs/msg/detail/metadata__functions.h"
#include "lidar_msgs/msg/detail/metadata__struct.h"


// Include directives for member types
// Member `computer_ip`
// Member `lidar_ip`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_msgs__msg__Metadata__init(message_memory);
}

void lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_fini_function(void * message_memory)
{
  lidar_msgs__msg__Metadata__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_member_array[4] = {
  {
    "computer_ip",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_msgs__msg__Metadata, computer_ip),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lidar_ip",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_msgs__msg__Metadata, lidar_ip),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu_port",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_msgs__msg__Metadata, imu_port),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lidar_port",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_msgs__msg__Metadata, lidar_port),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_members = {
  "lidar_msgs__msg",  // message namespace
  "Metadata",  // message name
  4,  // number of fields
  sizeof(lidar_msgs__msg__Metadata),
  lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_member_array,  // message members
  lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_init_function,  // function to initialize message memory (memory has to be allocated)
  lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_type_support_handle = {
  0,
  &lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_msgs, msg, Metadata)() {
  if (!lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_type_support_handle.typesupport_identifier) {
    lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lidar_msgs__msg__Metadata__rosidl_typesupport_introspection_c__Metadata_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
