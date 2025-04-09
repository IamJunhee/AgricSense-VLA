// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jackal_msgs:msg/DriveFeedback.idl
// generated code does not contain a copyright notice
#include "jackal_msgs/msg/detail/drive_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
jackal_msgs__msg__DriveFeedback__init(jackal_msgs__msg__DriveFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // current
  // duty_cycle
  // bridge_temperature
  // motor_temperature
  // measured_velocity
  // measured_travel
  // driver_fault
  return true;
}

void
jackal_msgs__msg__DriveFeedback__fini(jackal_msgs__msg__DriveFeedback * msg)
{
  if (!msg) {
    return;
  }
  // current
  // duty_cycle
  // bridge_temperature
  // motor_temperature
  // measured_velocity
  // measured_travel
  // driver_fault
}

bool
jackal_msgs__msg__DriveFeedback__are_equal(const jackal_msgs__msg__DriveFeedback * lhs, const jackal_msgs__msg__DriveFeedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // duty_cycle
  if (lhs->duty_cycle != rhs->duty_cycle) {
    return false;
  }
  // bridge_temperature
  if (lhs->bridge_temperature != rhs->bridge_temperature) {
    return false;
  }
  // motor_temperature
  if (lhs->motor_temperature != rhs->motor_temperature) {
    return false;
  }
  // measured_velocity
  if (lhs->measured_velocity != rhs->measured_velocity) {
    return false;
  }
  // measured_travel
  if (lhs->measured_travel != rhs->measured_travel) {
    return false;
  }
  // driver_fault
  if (lhs->driver_fault != rhs->driver_fault) {
    return false;
  }
  return true;
}

bool
jackal_msgs__msg__DriveFeedback__copy(
  const jackal_msgs__msg__DriveFeedback * input,
  jackal_msgs__msg__DriveFeedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current
  output->current = input->current;
  // duty_cycle
  output->duty_cycle = input->duty_cycle;
  // bridge_temperature
  output->bridge_temperature = input->bridge_temperature;
  // motor_temperature
  output->motor_temperature = input->motor_temperature;
  // measured_velocity
  output->measured_velocity = input->measured_velocity;
  // measured_travel
  output->measured_travel = input->measured_travel;
  // driver_fault
  output->driver_fault = input->driver_fault;
  return true;
}

jackal_msgs__msg__DriveFeedback *
jackal_msgs__msg__DriveFeedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__DriveFeedback * msg = (jackal_msgs__msg__DriveFeedback *)allocator.allocate(sizeof(jackal_msgs__msg__DriveFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jackal_msgs__msg__DriveFeedback));
  bool success = jackal_msgs__msg__DriveFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jackal_msgs__msg__DriveFeedback__destroy(jackal_msgs__msg__DriveFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jackal_msgs__msg__DriveFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jackal_msgs__msg__DriveFeedback__Sequence__init(jackal_msgs__msg__DriveFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__DriveFeedback * data = NULL;

  if (size) {
    data = (jackal_msgs__msg__DriveFeedback *)allocator.zero_allocate(size, sizeof(jackal_msgs__msg__DriveFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jackal_msgs__msg__DriveFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jackal_msgs__msg__DriveFeedback__fini(&data[i - 1]);
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
jackal_msgs__msg__DriveFeedback__Sequence__fini(jackal_msgs__msg__DriveFeedback__Sequence * array)
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
      jackal_msgs__msg__DriveFeedback__fini(&array->data[i]);
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

jackal_msgs__msg__DriveFeedback__Sequence *
jackal_msgs__msg__DriveFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__DriveFeedback__Sequence * array = (jackal_msgs__msg__DriveFeedback__Sequence *)allocator.allocate(sizeof(jackal_msgs__msg__DriveFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jackal_msgs__msg__DriveFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jackal_msgs__msg__DriveFeedback__Sequence__destroy(jackal_msgs__msg__DriveFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jackal_msgs__msg__DriveFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jackal_msgs__msg__DriveFeedback__Sequence__are_equal(const jackal_msgs__msg__DriveFeedback__Sequence * lhs, const jackal_msgs__msg__DriveFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jackal_msgs__msg__DriveFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jackal_msgs__msg__DriveFeedback__Sequence__copy(
  const jackal_msgs__msg__DriveFeedback__Sequence * input,
  jackal_msgs__msg__DriveFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jackal_msgs__msg__DriveFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jackal_msgs__msg__DriveFeedback * data =
      (jackal_msgs__msg__DriveFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jackal_msgs__msg__DriveFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jackal_msgs__msg__DriveFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jackal_msgs__msg__DriveFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
