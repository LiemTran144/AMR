// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__METADATA__STRUCT_H_
#define LIDAR_MSGS__MSG__DETAIL__METADATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'computer_ip'
// Member 'lidar_ip'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Metadata in the package lidar_msgs.
typedef struct lidar_msgs__msg__Metadata
{
  rosidl_runtime_c__String computer_ip;
  rosidl_runtime_c__String lidar_ip;
  int8_t imu_port;
  int8_t lidar_port;
} lidar_msgs__msg__Metadata;

// Struct for a sequence of lidar_msgs__msg__Metadata.
typedef struct lidar_msgs__msg__Metadata__Sequence
{
  lidar_msgs__msg__Metadata * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_msgs__msg__Metadata__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_MSGS__MSG__DETAIL__METADATA__STRUCT_H_
