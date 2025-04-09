// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jackal_msgs:msg/StopStatus.idl
// generated code does not contain a copyright notice
#include "jackal_msgs/msg/detail/stop_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
jackal_msgs__msg__StopStatus__init(jackal_msgs__msg__StopStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    jackal_msgs__msg__StopStatus__fini(msg);
    return false;
  }
  // stop_power_status
  // external_stop_present
  return true;
}

void
jackal_msgs__msg__StopStatus__fini(jackal_msgs__msg__StopStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // stop_power_status
  // external_stop_present
}

bool
jackal_msgs__msg__StopStatus__are_equal(const jackal_msgs__msg__StopStatus * lhs, const jackal_msgs__msg__StopStatus * rhs)
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
  // stop_power_status
  if (lhs->stop_power_status != rhs->stop_power_status) {
    return false;
  }
  // external_stop_present
  if (lhs->external_stop_present != rhs->external_stop_present) {
    return false;
  }
  return true;
}

bool
jackal_msgs__msg__StopStatus__copy(
  const jackal_msgs__msg__StopStatus * input,
  jackal_msgs__msg__StopStatus * output)
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
  // stop_power_status
  output->stop_power_status = input->stop_power_status;
  // external_stop_present
  output->external_stop_present = input->external_stop_present;
  return true;
}

jackal_msgs__msg__StopStatus *
jackal_msgs__msg__StopStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__StopStatus * msg = (jackal_msgs__msg__StopStatus *)allocator.allocate(sizeof(jackal_msgs__msg__StopStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jackal_msgs__msg__StopStatus));
  bool success = jackal_msgs__msg__StopStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jackal_msgs__msg__StopStatus__destroy(jackal_msgs__msg__StopStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jackal_msgs__msg__StopStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jackal_msgs__msg__StopStatus__Sequence__init(jackal_msgs__msg__StopStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__StopStatus * data = NULL;

  if (size) {
    data = (jackal_msgs__msg__StopStatus *)allocator.zero_allocate(size, sizeof(jackal_msgs__msg__StopStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jackal_msgs__msg__StopStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jackal_msgs__msg__StopStatus__fini(&data[i - 1]);
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
jackal_msgs__msg__StopStatus__Sequence__fini(jackal_msgs__msg__StopStatus__Sequence * array)
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
      jackal_msgs__msg__StopStatus__fini(&array->data[i]);
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

jackal_msgs__msg__StopStatus__Sequence *
jackal_msgs__msg__StopStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__StopStatus__Sequence * array = (jackal_msgs__msg__StopStatus__Sequence *)allocator.allocate(sizeof(jackal_msgs__msg__StopStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jackal_msgs__msg__StopStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jackal_msgs__msg__StopStatus__Sequence__destroy(jackal_msgs__msg__StopStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jackal_msgs__msg__StopStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jackal_msgs__msg__StopStatus__Sequence__are_equal(const jackal_msgs__msg__StopStatus__Sequence * lhs, const jackal_msgs__msg__StopStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jackal_msgs__msg__StopStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jackal_msgs__msg__StopStatus__Sequence__copy(
  const jackal_msgs__msg__StopStatus__Sequence * input,
  jackal_msgs__msg__StopStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jackal_msgs__msg__StopStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jackal_msgs__msg__StopStatus * data =
      (jackal_msgs__msg__StopStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jackal_msgs__msg__StopStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jackal_msgs__msg__StopStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jackal_msgs__msg__StopStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
