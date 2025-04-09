// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jackal_msgs:msg/Power.idl
// generated code does not contain a copyright notice
#include "jackal_msgs/msg/detail/power__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `measured_voltages`
// Member `measured_currents`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
jackal_msgs__msg__Power__init(jackal_msgs__msg__Power * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    jackal_msgs__msg__Power__fini(msg);
    return false;
  }
  // shore_power_connected
  // battery_connected
  // power_12v_user_nominal
  // charging_complete
  // measured_voltages
  if (!rosidl_runtime_c__float__Sequence__init(&msg->measured_voltages, 0)) {
    jackal_msgs__msg__Power__fini(msg);
    return false;
  }
  // measured_currents
  if (!rosidl_runtime_c__float__Sequence__init(&msg->measured_currents, 0)) {
    jackal_msgs__msg__Power__fini(msg);
    return false;
  }
  return true;
}

void
jackal_msgs__msg__Power__fini(jackal_msgs__msg__Power * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // shore_power_connected
  // battery_connected
  // power_12v_user_nominal
  // charging_complete
  // measured_voltages
  rosidl_runtime_c__float__Sequence__fini(&msg->measured_voltages);
  // measured_currents
  rosidl_runtime_c__float__Sequence__fini(&msg->measured_currents);
}

bool
jackal_msgs__msg__Power__are_equal(const jackal_msgs__msg__Power * lhs, const jackal_msgs__msg__Power * rhs)
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
  // shore_power_connected
  if (lhs->shore_power_connected != rhs->shore_power_connected) {
    return false;
  }
  // battery_connected
  if (lhs->battery_connected != rhs->battery_connected) {
    return false;
  }
  // power_12v_user_nominal
  if (lhs->power_12v_user_nominal != rhs->power_12v_user_nominal) {
    return false;
  }
  // charging_complete
  if (lhs->charging_complete != rhs->charging_complete) {
    return false;
  }
  // measured_voltages
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->measured_voltages), &(rhs->measured_voltages)))
  {
    return false;
  }
  // measured_currents
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->measured_currents), &(rhs->measured_currents)))
  {
    return false;
  }
  return true;
}

bool
jackal_msgs__msg__Power__copy(
  const jackal_msgs__msg__Power * input,
  jackal_msgs__msg__Power * output)
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
  // shore_power_connected
  output->shore_power_connected = input->shore_power_connected;
  // battery_connected
  output->battery_connected = input->battery_connected;
  // power_12v_user_nominal
  output->power_12v_user_nominal = input->power_12v_user_nominal;
  // charging_complete
  output->charging_complete = input->charging_complete;
  // measured_voltages
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->measured_voltages), &(output->measured_voltages)))
  {
    return false;
  }
  // measured_currents
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->measured_currents), &(output->measured_currents)))
  {
    return false;
  }
  return true;
}

jackal_msgs__msg__Power *
jackal_msgs__msg__Power__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__Power * msg = (jackal_msgs__msg__Power *)allocator.allocate(sizeof(jackal_msgs__msg__Power), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jackal_msgs__msg__Power));
  bool success = jackal_msgs__msg__Power__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jackal_msgs__msg__Power__destroy(jackal_msgs__msg__Power * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jackal_msgs__msg__Power__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jackal_msgs__msg__Power__Sequence__init(jackal_msgs__msg__Power__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__Power * data = NULL;

  if (size) {
    data = (jackal_msgs__msg__Power *)allocator.zero_allocate(size, sizeof(jackal_msgs__msg__Power), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jackal_msgs__msg__Power__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jackal_msgs__msg__Power__fini(&data[i - 1]);
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
jackal_msgs__msg__Power__Sequence__fini(jackal_msgs__msg__Power__Sequence * array)
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
      jackal_msgs__msg__Power__fini(&array->data[i]);
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

jackal_msgs__msg__Power__Sequence *
jackal_msgs__msg__Power__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__msg__Power__Sequence * array = (jackal_msgs__msg__Power__Sequence *)allocator.allocate(sizeof(jackal_msgs__msg__Power__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jackal_msgs__msg__Power__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jackal_msgs__msg__Power__Sequence__destroy(jackal_msgs__msg__Power__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jackal_msgs__msg__Power__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jackal_msgs__msg__Power__Sequence__are_equal(const jackal_msgs__msg__Power__Sequence * lhs, const jackal_msgs__msg__Power__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jackal_msgs__msg__Power__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jackal_msgs__msg__Power__Sequence__copy(
  const jackal_msgs__msg__Power__Sequence * input,
  jackal_msgs__msg__Power__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jackal_msgs__msg__Power);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jackal_msgs__msg__Power * data =
      (jackal_msgs__msg__Power *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jackal_msgs__msg__Power__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jackal_msgs__msg__Power__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jackal_msgs__msg__Power__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
