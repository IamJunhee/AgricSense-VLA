// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jackal_msgs:srv/SetDomainId.idl
// generated code does not contain a copyright notice
#include "jackal_msgs/srv/detail/set_domain_id__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
jackal_msgs__srv__SetDomainId_Request__init(jackal_msgs__srv__SetDomainId_Request * msg)
{
  if (!msg) {
    return false;
  }
  // domain_id
  return true;
}

void
jackal_msgs__srv__SetDomainId_Request__fini(jackal_msgs__srv__SetDomainId_Request * msg)
{
  if (!msg) {
    return;
  }
  // domain_id
}

bool
jackal_msgs__srv__SetDomainId_Request__are_equal(const jackal_msgs__srv__SetDomainId_Request * lhs, const jackal_msgs__srv__SetDomainId_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // domain_id
  if (lhs->domain_id != rhs->domain_id) {
    return false;
  }
  return true;
}

bool
jackal_msgs__srv__SetDomainId_Request__copy(
  const jackal_msgs__srv__SetDomainId_Request * input,
  jackal_msgs__srv__SetDomainId_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // domain_id
  output->domain_id = input->domain_id;
  return true;
}

jackal_msgs__srv__SetDomainId_Request *
jackal_msgs__srv__SetDomainId_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__srv__SetDomainId_Request * msg = (jackal_msgs__srv__SetDomainId_Request *)allocator.allocate(sizeof(jackal_msgs__srv__SetDomainId_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jackal_msgs__srv__SetDomainId_Request));
  bool success = jackal_msgs__srv__SetDomainId_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jackal_msgs__srv__SetDomainId_Request__destroy(jackal_msgs__srv__SetDomainId_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jackal_msgs__srv__SetDomainId_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jackal_msgs__srv__SetDomainId_Request__Sequence__init(jackal_msgs__srv__SetDomainId_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__srv__SetDomainId_Request * data = NULL;

  if (size) {
    data = (jackal_msgs__srv__SetDomainId_Request *)allocator.zero_allocate(size, sizeof(jackal_msgs__srv__SetDomainId_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jackal_msgs__srv__SetDomainId_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jackal_msgs__srv__SetDomainId_Request__fini(&data[i - 1]);
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
jackal_msgs__srv__SetDomainId_Request__Sequence__fini(jackal_msgs__srv__SetDomainId_Request__Sequence * array)
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
      jackal_msgs__srv__SetDomainId_Request__fini(&array->data[i]);
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

jackal_msgs__srv__SetDomainId_Request__Sequence *
jackal_msgs__srv__SetDomainId_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__srv__SetDomainId_Request__Sequence * array = (jackal_msgs__srv__SetDomainId_Request__Sequence *)allocator.allocate(sizeof(jackal_msgs__srv__SetDomainId_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jackal_msgs__srv__SetDomainId_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jackal_msgs__srv__SetDomainId_Request__Sequence__destroy(jackal_msgs__srv__SetDomainId_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jackal_msgs__srv__SetDomainId_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jackal_msgs__srv__SetDomainId_Request__Sequence__are_equal(const jackal_msgs__srv__SetDomainId_Request__Sequence * lhs, const jackal_msgs__srv__SetDomainId_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jackal_msgs__srv__SetDomainId_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jackal_msgs__srv__SetDomainId_Request__Sequence__copy(
  const jackal_msgs__srv__SetDomainId_Request__Sequence * input,
  jackal_msgs__srv__SetDomainId_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jackal_msgs__srv__SetDomainId_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jackal_msgs__srv__SetDomainId_Request * data =
      (jackal_msgs__srv__SetDomainId_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jackal_msgs__srv__SetDomainId_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jackal_msgs__srv__SetDomainId_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jackal_msgs__srv__SetDomainId_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
jackal_msgs__srv__SetDomainId_Response__init(jackal_msgs__srv__SetDomainId_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    jackal_msgs__srv__SetDomainId_Response__fini(msg);
    return false;
  }
  return true;
}

void
jackal_msgs__srv__SetDomainId_Response__fini(jackal_msgs__srv__SetDomainId_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
jackal_msgs__srv__SetDomainId_Response__are_equal(const jackal_msgs__srv__SetDomainId_Response * lhs, const jackal_msgs__srv__SetDomainId_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
jackal_msgs__srv__SetDomainId_Response__copy(
  const jackal_msgs__srv__SetDomainId_Response * input,
  jackal_msgs__srv__SetDomainId_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

jackal_msgs__srv__SetDomainId_Response *
jackal_msgs__srv__SetDomainId_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__srv__SetDomainId_Response * msg = (jackal_msgs__srv__SetDomainId_Response *)allocator.allocate(sizeof(jackal_msgs__srv__SetDomainId_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jackal_msgs__srv__SetDomainId_Response));
  bool success = jackal_msgs__srv__SetDomainId_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jackal_msgs__srv__SetDomainId_Response__destroy(jackal_msgs__srv__SetDomainId_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jackal_msgs__srv__SetDomainId_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jackal_msgs__srv__SetDomainId_Response__Sequence__init(jackal_msgs__srv__SetDomainId_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__srv__SetDomainId_Response * data = NULL;

  if (size) {
    data = (jackal_msgs__srv__SetDomainId_Response *)allocator.zero_allocate(size, sizeof(jackal_msgs__srv__SetDomainId_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jackal_msgs__srv__SetDomainId_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jackal_msgs__srv__SetDomainId_Response__fini(&data[i - 1]);
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
jackal_msgs__srv__SetDomainId_Response__Sequence__fini(jackal_msgs__srv__SetDomainId_Response__Sequence * array)
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
      jackal_msgs__srv__SetDomainId_Response__fini(&array->data[i]);
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

jackal_msgs__srv__SetDomainId_Response__Sequence *
jackal_msgs__srv__SetDomainId_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jackal_msgs__srv__SetDomainId_Response__Sequence * array = (jackal_msgs__srv__SetDomainId_Response__Sequence *)allocator.allocate(sizeof(jackal_msgs__srv__SetDomainId_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jackal_msgs__srv__SetDomainId_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jackal_msgs__srv__SetDomainId_Response__Sequence__destroy(jackal_msgs__srv__SetDomainId_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jackal_msgs__srv__SetDomainId_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jackal_msgs__srv__SetDomainId_Response__Sequence__are_equal(const jackal_msgs__srv__SetDomainId_Response__Sequence * lhs, const jackal_msgs__srv__SetDomainId_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jackal_msgs__srv__SetDomainId_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jackal_msgs__srv__SetDomainId_Response__Sequence__copy(
  const jackal_msgs__srv__SetDomainId_Response__Sequence * input,
  jackal_msgs__srv__SetDomainId_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jackal_msgs__srv__SetDomainId_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jackal_msgs__srv__SetDomainId_Response * data =
      (jackal_msgs__srv__SetDomainId_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jackal_msgs__srv__SetDomainId_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jackal_msgs__srv__SetDomainId_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jackal_msgs__srv__SetDomainId_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
