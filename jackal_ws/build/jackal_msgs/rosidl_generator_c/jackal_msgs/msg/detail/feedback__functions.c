// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jackal_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice
#include "jackal_msgs/msg/detail/feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `drivers`
#include "jackal_msgs/msg/detail/drive_feedback__functions.h"

bool
jackal_msgs__msg__Feedback__init(jackal_msgs__msg__Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    jackal_msgs__msg__Feedback__fini(msg);
    return false;
  }
  // drivers
  for (size_t i = 0; i < 2; ++i) {
    if (!jackal_msgs__msg__DriveFeedback__init(&msg->drivers[i])) {
      jackal_msgs__msg__Feedback__fini(msg);
      return false;
    }
  }
  // commanded_mode
  // actual_mode
  return true;
}

void
jackal_msgs__msg__Feedback__fini(jackal_msgs__msg__Feedback * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // drivers
  for (size_t i = 0; i < 2; ++i) {
    jackal_msgs__msg__DriveFeedback__fini(&msg->drivers[i]);
  }
  // commanded_mode
  // actual_mode
}

bool
jackal_msgs__msg__Feedback__are_equal(const jackal_msgs__msg__Feedback * lhs, const jackal_msgs__msg__Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // drivers
  for (size_t i = 0; i < 2; ++i) {
    if (!jackal_msgs__msg__DriveFeedback__are_equal(
        &(lhs->drivers[i]), &(rhs->drivers[i])))
    {
      return false;
    }
  }
  // commanded_mode
  if (lhs->commanded_mode != rhs->commanded_mode) {
    return false;
  }
  // actual_mode
  if (lhs->actual_mode != rhs->actual_mode) {
    return false;
  }
  return true;
}

bool
jackal_msgs__msg__Feedback__copy(
  const jackal_msgs__msg__Feedback * input,
  jackal_msgs__msg__Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // drivers
  for (size_t i = 0; i < 2; ++i) {
    if (!jackal_msgs__msg__DriveFeedback__copy(
        &(input->drivers[i]), &(output->drivers[i])))
    {
      return false;
    }
  }
  // commanded_mode
  output->commanded_mode = input->commanded_mode;
  // actual_mode
  output->actual_mode = input->actual_mode;
  return true;
}

jackal_msgs__msg__Feedback *
jackal_msgs__msg__Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__Feedback * msg = (jackal_msgs__msg__Feedback *)allocator.allocate(sizeof(jackal_msgs__msg__Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jackal_msgs__msg__Feedback));
  bool success = jackal_msgs__msg__Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jackal_msgs__msg__Feedback__destroy(jackal_msgs__msg__Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jackal_msgs__msg__Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jackal_msgs__msg__Feedback__Sequence__init(jackal_msgs__msg__Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__Feedback * data = NULL;

  if (size) {
    data = (jackal_msgs__msg__Feedback *)allocator.zero_allocate(size, sizeof(jackal_msgs__msg__Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jackal_msgs__msg__Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jackal_msgs__msg__Feedback__fini(&data[i - 1]);
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
jackal_msgs__msg__Feedback__Sequence__fini(jackal_msgs__msg__Feedback__Sequence * array)
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
      jackal_msgs__msg__Feedback__fini(&array->data[i]);
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

jackal_msgs__msg__Feedback__Sequence *
jackal_msgs__msg__Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__Feedback__Sequence * array = (jackal_msgs__msg__Feedback__Sequence *)allocator.allocate(sizeof(jackal_msgs__msg__Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jackal_msgs__msg__Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jackal_msgs__msg__Feedback__Sequence__destroy(jackal_msgs__msg__Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jackal_msgs__msg__Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jackal_msgs__msg__Feedback__Sequence__are_equal(const jackal_msgs__msg__Feedback__Sequence * lhs, const jackal_msgs__msg__Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jackal_msgs__msg__Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jackal_msgs__msg__Feedback__Sequence__copy(
  const jackal_msgs__msg__Feedback__Sequence * input,
  jackal_msgs__msg__Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jackal_msgs__msg__Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jackal_msgs__msg__Feedback * data =
      (jackal_msgs__msg__Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jackal_msgs__msg__Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jackal_msgs__msg__Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jackal_msgs__msg__Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
