// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosbasic_msgs:srv/AddTwoInts.idl
// generated code does not contain a copyright notice
#include "rosbasic_msgs/srv/detail/add_two_ints__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
rosbasic_msgs__srv__AddTwoInts_Request__init(rosbasic_msgs__srv__AddTwoInts_Request * msg)
{
  if (!msg) {
    return false;
  }
  // c
  // d
  return true;
}

void
rosbasic_msgs__srv__AddTwoInts_Request__fini(rosbasic_msgs__srv__AddTwoInts_Request * msg)
{
  if (!msg) {
    return;
  }
  // c
  // d
}

bool
rosbasic_msgs__srv__AddTwoInts_Request__are_equal(const rosbasic_msgs__srv__AddTwoInts_Request * lhs, const rosbasic_msgs__srv__AddTwoInts_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // c
  if (lhs->c != rhs->c) {
    return false;
  }
  // d
  if (lhs->d != rhs->d) {
    return false;
  }
  return true;
}

bool
rosbasic_msgs__srv__AddTwoInts_Request__copy(
  const rosbasic_msgs__srv__AddTwoInts_Request * input,
  rosbasic_msgs__srv__AddTwoInts_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // c
  output->c = input->c;
  // d
  output->d = input->d;
  return true;
}

rosbasic_msgs__srv__AddTwoInts_Request *
rosbasic_msgs__srv__AddTwoInts_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_msgs__srv__AddTwoInts_Request * msg = (rosbasic_msgs__srv__AddTwoInts_Request *)allocator.allocate(sizeof(rosbasic_msgs__srv__AddTwoInts_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosbasic_msgs__srv__AddTwoInts_Request));
  bool success = rosbasic_msgs__srv__AddTwoInts_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosbasic_msgs__srv__AddTwoInts_Request__destroy(rosbasic_msgs__srv__AddTwoInts_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosbasic_msgs__srv__AddTwoInts_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosbasic_msgs__srv__AddTwoInts_Request__Sequence__init(rosbasic_msgs__srv__AddTwoInts_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_msgs__srv__AddTwoInts_Request * data = NULL;

  if (size) {
    data = (rosbasic_msgs__srv__AddTwoInts_Request *)allocator.zero_allocate(size, sizeof(rosbasic_msgs__srv__AddTwoInts_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosbasic_msgs__srv__AddTwoInts_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosbasic_msgs__srv__AddTwoInts_Request__fini(&data[i - 1]);
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
rosbasic_msgs__srv__AddTwoInts_Request__Sequence__fini(rosbasic_msgs__srv__AddTwoInts_Request__Sequence * array)
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
      rosbasic_msgs__srv__AddTwoInts_Request__fini(&array->data[i]);
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

rosbasic_msgs__srv__AddTwoInts_Request__Sequence *
rosbasic_msgs__srv__AddTwoInts_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_msgs__srv__AddTwoInts_Request__Sequence * array = (rosbasic_msgs__srv__AddTwoInts_Request__Sequence *)allocator.allocate(sizeof(rosbasic_msgs__srv__AddTwoInts_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosbasic_msgs__srv__AddTwoInts_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosbasic_msgs__srv__AddTwoInts_Request__Sequence__destroy(rosbasic_msgs__srv__AddTwoInts_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosbasic_msgs__srv__AddTwoInts_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosbasic_msgs__srv__AddTwoInts_Request__Sequence__are_equal(const rosbasic_msgs__srv__AddTwoInts_Request__Sequence * lhs, const rosbasic_msgs__srv__AddTwoInts_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosbasic_msgs__srv__AddTwoInts_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosbasic_msgs__srv__AddTwoInts_Request__Sequence__copy(
  const rosbasic_msgs__srv__AddTwoInts_Request__Sequence * input,
  rosbasic_msgs__srv__AddTwoInts_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosbasic_msgs__srv__AddTwoInts_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosbasic_msgs__srv__AddTwoInts_Request * data =
      (rosbasic_msgs__srv__AddTwoInts_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosbasic_msgs__srv__AddTwoInts_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosbasic_msgs__srv__AddTwoInts_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosbasic_msgs__srv__AddTwoInts_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
rosbasic_msgs__srv__AddTwoInts_Response__init(rosbasic_msgs__srv__AddTwoInts_Response * msg)
{
  if (!msg) {
    return false;
  }
  // sum
  return true;
}

void
rosbasic_msgs__srv__AddTwoInts_Response__fini(rosbasic_msgs__srv__AddTwoInts_Response * msg)
{
  if (!msg) {
    return;
  }
  // sum
}

bool
rosbasic_msgs__srv__AddTwoInts_Response__are_equal(const rosbasic_msgs__srv__AddTwoInts_Response * lhs, const rosbasic_msgs__srv__AddTwoInts_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sum
  if (lhs->sum != rhs->sum) {
    return false;
  }
  return true;
}

bool
rosbasic_msgs__srv__AddTwoInts_Response__copy(
  const rosbasic_msgs__srv__AddTwoInts_Response * input,
  rosbasic_msgs__srv__AddTwoInts_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // sum
  output->sum = input->sum;
  return true;
}

rosbasic_msgs__srv__AddTwoInts_Response *
rosbasic_msgs__srv__AddTwoInts_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_msgs__srv__AddTwoInts_Response * msg = (rosbasic_msgs__srv__AddTwoInts_Response *)allocator.allocate(sizeof(rosbasic_msgs__srv__AddTwoInts_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosbasic_msgs__srv__AddTwoInts_Response));
  bool success = rosbasic_msgs__srv__AddTwoInts_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosbasic_msgs__srv__AddTwoInts_Response__destroy(rosbasic_msgs__srv__AddTwoInts_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosbasic_msgs__srv__AddTwoInts_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosbasic_msgs__srv__AddTwoInts_Response__Sequence__init(rosbasic_msgs__srv__AddTwoInts_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_msgs__srv__AddTwoInts_Response * data = NULL;

  if (size) {
    data = (rosbasic_msgs__srv__AddTwoInts_Response *)allocator.zero_allocate(size, sizeof(rosbasic_msgs__srv__AddTwoInts_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosbasic_msgs__srv__AddTwoInts_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosbasic_msgs__srv__AddTwoInts_Response__fini(&data[i - 1]);
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
rosbasic_msgs__srv__AddTwoInts_Response__Sequence__fini(rosbasic_msgs__srv__AddTwoInts_Response__Sequence * array)
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
      rosbasic_msgs__srv__AddTwoInts_Response__fini(&array->data[i]);
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

rosbasic_msgs__srv__AddTwoInts_Response__Sequence *
rosbasic_msgs__srv__AddTwoInts_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbasic_msgs__srv__AddTwoInts_Response__Sequence * array = (rosbasic_msgs__srv__AddTwoInts_Response__Sequence *)allocator.allocate(sizeof(rosbasic_msgs__srv__AddTwoInts_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosbasic_msgs__srv__AddTwoInts_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosbasic_msgs__srv__AddTwoInts_Response__Sequence__destroy(rosbasic_msgs__srv__AddTwoInts_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosbasic_msgs__srv__AddTwoInts_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosbasic_msgs__srv__AddTwoInts_Response__Sequence__are_equal(const rosbasic_msgs__srv__AddTwoInts_Response__Sequence * lhs, const rosbasic_msgs__srv__AddTwoInts_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosbasic_msgs__srv__AddTwoInts_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosbasic_msgs__srv__AddTwoInts_Response__Sequence__copy(
  const rosbasic_msgs__srv__AddTwoInts_Response__Sequence * input,
  rosbasic_msgs__srv__AddTwoInts_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosbasic_msgs__srv__AddTwoInts_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosbasic_msgs__srv__AddTwoInts_Response * data =
      (rosbasic_msgs__srv__AddTwoInts_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosbasic_msgs__srv__AddTwoInts_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosbasic_msgs__srv__AddTwoInts_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosbasic_msgs__srv__AddTwoInts_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
