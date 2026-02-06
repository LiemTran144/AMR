// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosbasic_msgs:srv/Multi.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MSGS__SRV__DETAIL__MULTI__STRUCT_H_
#define ROSBASIC_MSGS__SRV__DETAIL__MULTI__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Multi in the package rosbasic_msgs.
typedef struct rosbasic_msgs__srv__Multi_Request
{
  int64_t a;
  int64_t b;
} rosbasic_msgs__srv__Multi_Request;

// Struct for a sequence of rosbasic_msgs__srv__Multi_Request.
typedef struct rosbasic_msgs__srv__Multi_Request__Sequence
{
  rosbasic_msgs__srv__Multi_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbasic_msgs__srv__Multi_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Multi in the package rosbasic_msgs.
typedef struct rosbasic_msgs__srv__Multi_Response
{
  int64_t mul;
} rosbasic_msgs__srv__Multi_Response;

// Struct for a sequence of rosbasic_msgs__srv__Multi_Response.
typedef struct rosbasic_msgs__srv__Multi_Response__Sequence
{
  rosbasic_msgs__srv__Multi_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbasic_msgs__srv__Multi_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSBASIC_MSGS__SRV__DETAIL__MULTI__STRUCT_H_
