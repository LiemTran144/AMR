// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosbasic_move_msgs:srv/Control.idl
// generated code does not contain a copyright notice
#include "rosbasic_move_msgs/srv/detail/control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
rosbasic_move_msgs__srv__Control_Request__init(rosbasic_move_msgs__srv__Control_Request * msg)
{
  if (!msg) {
    return false;
  }
  // quangduong
  // khoangcach
  return true;
}

void
rosbasic_move_msgs__srv__Control_Request__fini(rosbasic_move_msgs__srv__Control_Request * msg)
{
  if (!msg) {
    return;
  }
  // quangduong
  // khoangcach
}

bool
rosbasic_move_msgs__srv__Control_Request__are_equal(const rosbasic_move_msgs__srv__Control_Request * lhs, const rosbasic_move_msgs__srv__Control_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // quangduong
  if (lhs->quangduong != rhs->quangduong) {
    return false;
  }
  // khoangcach
  if (lhs->khoangcach != rhs->khoangcach) {
    return false;
  }
  return true;
}

bool
rosbasic_move_msgs__srv__Control_Request__copy(
  const rosbasic_move_msgs__srv__Control_Request * input,
  rosbasic_move_msgs__srv__Control_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // quangduong
  output->quangduong = input->quangduong;
  // khoangcach
  output->khoangcach = input->khoangcach;
  return true;
}

rosbasic_move_msgs__srv__Control_Request *
rosbasic_move_msgs__srv__Control_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_move_msgs__srv__Control_Request * msg = (rosbasic_move_msgs__srv__Control_Request *)allocator.allocate(sizeof(rosbasic_move_msgs__srv__Control_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosbasic_move_msgs__srv__Control_Request));
  bool success = rosbasic_move_msgs__srv__Control_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosbasic_move_msgs__srv__Control_Request__destroy(rosbasic_move_msgs__srv__Control_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosbasic_move_msgs__srv__Control_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosbasic_move_msgs__srv__Control_Request__Sequence__init(rosbasic_move_msgs__srv__Control_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_move_msgs__srv__Control_Request * data = NULL;

  if (size) {
    data = (rosbasic_move_msgs__srv__Control_Request *)allocator.zero_allocate(size, sizeof(rosbasic_move_msgs__srv__Control_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosbasic_move_msgs__srv__Control_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosbasic_move_msgs__srv__Control_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rosbasic_move_msgs__srv__Control_Request__Sequence__fini(rosbasic_move_msgs__srv__Control_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rosbasic_move_msgs__srv__Control_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rosbasic_move_msgs__srv__Control_Request__Sequence *
rosbasic_move_msgs__srv__Control_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_move_msgs__srv__Control_Request__Sequence * array = (rosbasic_move_msgs__srv__Control_Request__Sequence *)allocator.allocate(sizeof(rosbasic_move_msgs__srv__Control_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosbasic_move_msgs__srv__Control_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosbasic_move_msgs__srv__Control_Request__Sequence__destroy(rosbasic_move_msgs__srv__Control_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosbasic_move_msgs__srv__Control_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosbasic_move_msgs__srv__Control_Request__Sequence__are_equal(const rosbasic_move_msgs__srv__Control_Request__Sequence * lhs, const rosbasic_move_msgs__srv__Control_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosbasic_move_msgs__srv__Control_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosbasic_move_msgs__srv__Control_Request__Sequence__copy(
  const rosbasic_move_msgs__srv__Control_Request__Sequence * input,
  rosbasic_move_msgs__srv__Control_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosbasic_move_msgs__srv__Control_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosbasic_move_msgs__srv__Control_Request * data =
      (rosbasic_move_msgs__srv__Control_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosbasic_move_msgs__srv__Control_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosbasic_move_msgs__srv__Control_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosbasic_move_msgs__srv__Control_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `notification`
#include "rosidl_runtime_c/string_functions.h"

bool
rosbasic_move_msgs__srv__Control_Response__init(rosbasic_move_msgs__srv__Control_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // notification
  if (!rosidl_runtime_c__String__init(&msg->notification)) {
    rosbasic_move_msgs__srv__Control_Response__fini(msg);
    return false;
  }
  return true;
}

void
rosbasic_move_msgs__srv__Control_Response__fini(rosbasic_move_msgs__srv__Control_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // notification
  rosidl_runtime_c__String__fini(&msg->notification);
}

bool
rosbasic_move_msgs__srv__Control_Response__are_equal(const rosbasic_move_msgs__srv__Control_Response * lhs, const rosbasic_move_msgs__srv__Control_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // notification
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->notification), &(rhs->notification)))
  {
    return false;
  }
  return true;
}

bool
rosbasic_move_msgs__srv__Control_Response__copy(
  const rosbasic_move_msgs__srv__Control_Response * input,
  rosbasic_move_msgs__srv__Control_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // notification
  if (!rosidl_runtime_c__String__copy(
      &(input->notification), &(output->notification)))
  {
    return false;
  }
  return true;
}

rosbasic_move_msgs__srv__Control_Response *
rosbasic_move_msgs__srv__Control_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_move_msgs__srv__Control_Response * msg = (rosbasic_move_msgs__srv__Control_Response *)allocator.allocate(sizeof(rosbasic_move_msgs__srv__Control_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosbasic_move_msgs__srv__Control_Response));
  bool success = rosbasic_move_msgs__srv__Control_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosbasic_move_msgs__srv__Control_Response__destroy(rosbasic_move_msgs__srv__Control_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosbasic_move_msgs__srv__Control_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosbasic_move_msgs__srv__Control_Response__Sequence__init(rosbasic_move_msgs__srv__Control_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_move_msgs__srv__Control_Response * data = NULL;

  if (size) {
    data = (rosbasic_move_msgs__srv__Control_Response *)allocator.zero_allocate(size, sizeof(rosbasic_move_msgs__srv__Control_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosbasic_move_msgs__srv__Control_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosbasic_move_msgs__srv__Control_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rosbasic_move_msgs__srv__Control_Response__Sequence__fini(rosbasic_move_msgs__srv__Control_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rosbasic_move_msgs__srv__Control_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rosbasic_move_msgs__srv__Control_Response__Sequence *
rosbasic_move_msgs__srv__Control_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_move_msgs__srv__Control_Response__Sequence * array = (rosbasic_move_msgs__srv__Control_Response__Sequence *)allocator.allocate(sizeof(rosbasic_move_msgs__srv__Control_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosbasic_move_msgs__srv__Control_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosbasic_move_msgs__srv__Control_Response__Sequence__destroy(rosbasic_move_msgs__srv__Control_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosbasic_move_msgs__srv__Control_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosbasic_move_msgs__srv__Control_Response__Sequence__are_equal(const rosbasic_move_msgs__srv__Control_Response__Sequence * lhs, const rosbasic_move_msgs__srv__Control_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosbasic_move_msgs__srv__Control_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosbasic_move_msgs__srv__Control_Response__Sequence__copy(
  const rosbasic_move_msgs__srv__Control_Response__Sequence * input,
  rosbasic_move_msgs__srv__Control_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosbasic_move_msgs__srv__Control_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosbasic_move_msgs__srv__Control_Response * data =
      (rosbasic_move_msgs__srv__Control_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosbasic_move_msgs__srv__Control_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosbasic_move_msgs__srv__Control_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosbasic_move_msgs__srv__Control_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
