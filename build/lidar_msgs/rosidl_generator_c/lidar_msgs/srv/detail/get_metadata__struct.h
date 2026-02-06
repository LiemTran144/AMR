// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_msgs:srv/GetMetadata.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__SRV__DETAIL__GET_METADATA__STRUCT_H_
#define LIDAR_MSGS__SRV__DETAIL__GET_METADATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetMetadata in the package lidar_msgs.
typedef struct lidar_msgs__srv__GetMetadata_Request
{
  uint8_t structure_needs_at_least_one_member;
} lidar_msgs__srv__GetMetadata_Request;

// Struct for a sequence of lidar_msgs__srv__GetMetadata_Request.
typedef struct lidar_msgs__srv__GetMetadata_Request__Sequence
{
  lidar_msgs__srv__GetMetadata_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_msgs__srv__GetMetadata_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'metadata'
#include "lidar_msgs/msg/detail/metadata__struct.h"

/// Struct defined in srv/GetMetadata in the package lidar_msgs.
typedef struct lidar_msgs__srv__GetMetadata_Response
{
  lidar_msgs__msg__Metadata metadata;
} lidar_msgs__srv__GetMetadata_Response;

// Struct for a sequence of lidar_msgs__srv__GetMetadata_Response.
typedef struct lidar_msgs__srv__GetMetadata_Response__Sequence
{
  lidar_msgs__srv__GetMetadata_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_msgs__srv__GetMetadata_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_MSGS__SRV__DETAIL__GET_METADATA__STRUCT_H_
