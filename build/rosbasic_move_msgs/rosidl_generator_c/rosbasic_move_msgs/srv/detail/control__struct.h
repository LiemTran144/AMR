// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosbasic_move_msgs:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__STRUCT_H_
#define ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Control in the package rosbasic_move_msgs.
typedef struct rosbasic_move_msgs__srv__Control_Request
{
  float quangduong;
  float khoangcach;
} rosbasic_move_msgs__srv__Control_Request;

// Struct for a sequence of rosbasic_move_msgs__srv__Control_Request.
typedef struct rosbasic_move_msgs__srv__Control_Request__Sequence
{
  rosbasic_move_msgs__srv__Control_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbasic_move_msgs__srv__Control_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'notification'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Control in the package rosbasic_move_msgs.
typedef struct rosbasic_move_msgs__srv__Control_Response
{
  bool success;
  rosidl_runtime_c__String notification;
} rosbasic_move_msgs__srv__Control_Response;

// Struct for a sequence of rosbasic_move_msgs__srv__Control_Response.
typedef struct rosbasic_move_msgs__srv__Control_Response__Sequence
{
  rosbasic_move_msgs__srv__Control_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbasic_move_msgs__srv__Control_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__STRUCT_H_
