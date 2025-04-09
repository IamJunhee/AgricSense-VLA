// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jackal_msgs:msg/Drive.idl
// generated code does not contain a copyright notice
#include "jackal_msgs/msg/detail/drive__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
jackal_msgs__msg__Drive__init(jackal_msgs__msg__Drive * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  // drivers
  return true;
}

void
jackal_msgs__msg__Drive__fini(jackal_msgs__msg__Drive * msg)
{
  if (!msg) {
    return;
  }
  // mode
  // drivers
}

bool
jackal_msgs__msg__Drive__are_equal(const jackal_msgs__msg__Drive * lhs, const jackal_msgs__msg__Drive * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // drivers
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->drivers[i] != rhs->drivers[i]) {
      return false;
    }
  }
  return true;
}

bool
jackal_msgs__msg__Drive__copy(
  const jackal_msgs__msg__Drive * input,
  jackal_msgs__msg__Drive * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  // drivers
  for (size_t i = 0; i < 2; ++i) {
    output->drivers[i] = input->drivers[i];
  }
  return true;
}

jackal_msgs__msg__Drive *
jackal_msgs__msg__Drive__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__Drive * msg = (jackal_msgs__msg__Drive *)allocator.allocate(sizeof(jackal_msgs__msg__Drive), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jackal_msgs__msg__Drive));
  bool success = jackal_msgs__msg__Drive__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jackal_msgs__msg__Drive__destroy(jackal_msgs__msg__Drive * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jackal_msgs__msg__Drive__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jackal_msgs__msg__Drive__Sequence__init(jackal_msgs__msg__Drive__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__Drive * data = NULL;

  if (size) {
    data = (jackal_msgs__msg__Drive *)allocator.zero_allocate(size, sizeof(jackal_msgs__msg__Drive), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jackal_msgs__msg__Drive__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jackal_msgs__msg__Drive__fini(&data[i - 1]);
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
jackal_msgs__msg__Drive__Sequence__fini(jackal_msgs__msg__Drive__Sequence * array)
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
      jackal_msgs__msg__Drive__fini(&array->data[i]);
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

jackal_msgs__msg__Drive__Sequence *
jackal_msgs__msg__Drive__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__Drive__Sequence * array = (jackal_msgs__msg__Drive__Sequence *)allocator.allocate(sizeof(jackal_msgs__msg__Drive__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jackal_msgs__msg__Drive__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jackal_msgs__msg__Drive__Sequence__destroy(jackal_msgs__msg__Drive__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jackal_msgs__msg__Drive__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jackal_msgs__msg__Drive__Sequence__are_equal(const jackal_msgs__msg__Drive__Sequence * lhs, const jackal_msgs__msg__Drive__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jackal_msgs__msg__Drive__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jackal_msgs__msg__Drive__Sequence__copy(
  const jackal_msgs__msg__Drive__Sequence * input,
  jackal_msgs__msg__Drive__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jackal_msgs__msg__Drive);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jackal_msgs__msg__Drive * data =
      (jackal_msgs__msg__Drive *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jackal_msgs__msg__Drive__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jackal_msgs__msg__Drive__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jackal_msgs__msg__Drive__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
