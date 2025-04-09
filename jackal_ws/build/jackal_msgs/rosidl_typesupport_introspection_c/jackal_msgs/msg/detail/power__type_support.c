// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from jackal_msgs:msg/Power.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "jackal_msgs/msg/detail/power__rosidl_typesupport_introspection_c.h"
#include "jackal_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "jackal_msgs/msg/detail/power__functions.h"
#include "jackal_msgs/msg/detail/power__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `measured_voltages`
// Member `measured_currents`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jackal_msgs__msg__Power__init(message_memory);
}

void jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_fini_function(void * message_memory)
{
  jackal_msgs__msg__Power__fini(message_memory);
}

size_t jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__size_function__Power__measured_voltages(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_const_function__Power__measured_voltages(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_function__Power__measured_voltages(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__fetch_function__Power__measured_voltages(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_const_function__Power__measured_voltages(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__assign_function__Power__measured_voltages(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_function__Power__measured_voltages(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__resize_function__Power__measured_voltages(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__size_function__Power__measured_currents(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_const_function__Power__measured_currents(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_function__Power__measured_currents(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__fetch_function__Power__measured_currents(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_const_function__Power__measured_currents(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__assign_function__Power__measured_currents(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_function__Power__measured_currents(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__resize_function__Power__measured_currents(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Power, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "shore_power_connected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Power, shore_power_connected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_connected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Power, battery_connected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "power_12v_user_nominal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Power, power_12v_user_nominal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "charging_complete",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Power, charging_complete),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "measured_voltages",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Power, measured_voltages),  // bytes offset in struct
    NULL,  // default value
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__size_function__Power__measured_voltages,  // size() function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_const_function__Power__measured_voltages,  // get_const(index) function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_function__Power__measured_voltages,  // get(index) function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__fetch_function__Power__measured_voltages,  // fetch(index, &value) function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__assign_function__Power__measured_voltages,  // assign(index, value) function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__resize_function__Power__measured_voltages  // resize(index) function pointer
  },
  {
    "measured_currents",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Power, measured_currents),  // bytes offset in struct
    NULL,  // default value
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__size_function__Power__measured_currents,  // size() function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_const_function__Power__measured_currents,  // get_const(index) function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__get_function__Power__measured_currents,  // get(index) function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__fetch_function__Power__measured_currents,  // fetch(index, &value) function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__assign_function__Power__measured_currents,  // assign(index, value) function pointer
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__resize_function__Power__measured_currents  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_message_members = {
  "jackal_msgs__msg",  // message namespace
  "Power",  // message name
  7,  // number of fields
  sizeof(jackal_msgs__msg__Power),
  jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_message_member_array,  // message members
  jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_init_function,  // function to initialize message memory (memory has to be allocated)
  jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_message_type_support_handle = {
  0,
  &jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jackal_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jackal_msgs, msg, Power)() {
  jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_message_type_support_handle.typesupport_identifier) {
    jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &jackal_msgs__msg__Power__rosidl_typesupport_introspection_c__Power_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
