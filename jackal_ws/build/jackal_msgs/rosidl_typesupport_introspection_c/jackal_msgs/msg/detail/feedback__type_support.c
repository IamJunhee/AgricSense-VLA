// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from jackal_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "jackal_msgs/msg/detail/feedback__rosidl_typesupport_introspection_c.h"
#include "jackal_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "jackal_msgs/msg/detail/feedback__functions.h"
#include "jackal_msgs/msg/detail/feedback__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `drivers`
#include "jackal_msgs/msg/drive_feedback.h"
// Member `drivers`
#include "jackal_msgs/msg/detail/drive_feedback__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jackal_msgs__msg__Feedback__init(message_memory);
}

void jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_fini_function(void * message_memory)
{
  jackal_msgs__msg__Feedback__fini(message_memory);
}

size_t jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__size_function__Feedback__drivers(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__get_const_function__Feedback__drivers(
  const void * untyped_member, size_t index)
{
  const jackal_msgs__msg__DriveFeedback * member =
    (const jackal_msgs__msg__DriveFeedback *)(untyped_member);
  return &member[index];
}

void * jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__get_function__Feedback__drivers(
  void * untyped_member, size_t index)
{
  jackal_msgs__msg__DriveFeedback * member =
    (jackal_msgs__msg__DriveFeedback *)(untyped_member);
  return &member[index];
}

void jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__fetch_function__Feedback__drivers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const jackal_msgs__msg__DriveFeedback * item =
    ((const jackal_msgs__msg__DriveFeedback *)
    jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__get_const_function__Feedback__drivers(untyped_member, index));
  jackal_msgs__msg__DriveFeedback * value =
    (jackal_msgs__msg__DriveFeedback *)(untyped_value);
  *value = *item;
}

void jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__assign_function__Feedback__drivers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  jackal_msgs__msg__DriveFeedback * item =
    ((jackal_msgs__msg__DriveFeedback *)
    jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__get_function__Feedback__drivers(untyped_member, index));
  const jackal_msgs__msg__DriveFeedback * value =
    (const jackal_msgs__msg__DriveFeedback *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Feedback, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "drivers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Feedback, drivers),  // bytes offset in struct
    NULL,  // default value
    jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__size_function__Feedback__drivers,  // size() function pointer
    jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__get_const_function__Feedback__drivers,  // get_const(index) function pointer
    jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__get_function__Feedback__drivers,  // get(index) function pointer
    jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__fetch_function__Feedback__drivers,  // fetch(index, &value) function pointer
    jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__assign_function__Feedback__drivers,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "commanded_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Feedback, commanded_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actual_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs__msg__Feedback, actual_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_members = {
  "jackal_msgs__msg",  // message namespace
  "Feedback",  // message name
  4,  // number of fields
  sizeof(jackal_msgs__msg__Feedback),
  jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_member_array,  // message members
  jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_type_support_handle = {
  0,
  &jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jackal_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jackal_msgs, msg, Feedback)() {
  jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jackal_msgs, msg, DriveFeedback)();
  if (!jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_type_support_handle.typesupport_identifier) {
    jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &jackal_msgs__msg__Feedback__rosidl_typesupport_introspection_c__Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
