// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from jackal_msgs:msg/DriveFeedback.idl
// generated code does not contain a copyright notice
#include "jackal_msgs/msg/detail/drive_feedback__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "jackal_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "jackal_msgs/msg/detail/drive_feedback__struct.h"
#include "jackal_msgs/msg/detail/drive_feedback__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _DriveFeedback__ros_msg_type = jackal_msgs__msg__DriveFeedback;

static bool _DriveFeedback__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DriveFeedback__ros_msg_type * ros_message = static_cast<const _DriveFeedback__ros_msg_type *>(untyped_ros_message);
  // Field name: current
  {
    cdr << ros_message->current;
  }

  // Field name: duty_cycle
  {
    cdr << ros_message->duty_cycle;
  }

  // Field name: bridge_temperature
  {
    cdr << ros_message->bridge_temperature;
  }

  // Field name: motor_temperature
  {
    cdr << ros_message->motor_temperature;
  }

  // Field name: measured_velocity
  {
    cdr << ros_message->measured_velocity;
  }

  // Field name: measured_travel
  {
    cdr << ros_message->measured_travel;
  }

  // Field name: driver_fault
  {
    cdr << (ros_message->driver_fault ? true : false);
  }

  return true;
}

static bool _DriveFeedback__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DriveFeedback__ros_msg_type * ros_message = static_cast<_DriveFeedback__ros_msg_type *>(untyped_ros_message);
  // Field name: current
  {
    cdr >> ros_message->current;
  }

  // Field name: duty_cycle
  {
    cdr >> ros_message->duty_cycle;
  }

  // Field name: bridge_temperature
  {
    cdr >> ros_message->bridge_temperature;
  }

  // Field name: motor_temperature
  {
    cdr >> ros_message->motor_temperature;
  }

  // Field name: measured_velocity
  {
    cdr >> ros_message->measured_velocity;
  }

  // Field name: measured_travel
  {
    cdr >> ros_message->measured_travel;
  }

  // Field name: driver_fault
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->driver_fault = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_jackal_msgs
size_t get_serialized_size_jackal_msgs__msg__DriveFeedback(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DriveFeedback__ros_msg_type * ros_message = static_cast<const _DriveFeedback__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name current
  {
    size_t item_size = sizeof(ros_message->current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name duty_cycle
  {
    size_t item_size = sizeof(ros_message->duty_cycle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bridge_temperature
  {
    size_t item_size = sizeof(ros_message->bridge_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor_temperature
  {
    size_t item_size = sizeof(ros_message->motor_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name measured_velocity
  {
    size_t item_size = sizeof(ros_message->measured_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name measured_travel
  {
    size_t item_size = sizeof(ros_message->measured_travel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name driver_fault
  {
    size_t item_size = sizeof(ros_message->driver_fault);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _DriveFeedback__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_jackal_msgs__msg__DriveFeedback(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_jackal_msgs
size_t max_serialized_size_jackal_msgs__msg__DriveFeedback(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: duty_cycle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: bridge_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: motor_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: measured_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: measured_travel
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: driver_fault
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = jackal_msgs__msg__DriveFeedback;
    is_plain =
      (
      offsetof(DataType, driver_fault) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _DriveFeedback__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_jackal_msgs__msg__DriveFeedback(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DriveFeedback = {
  "jackal_msgs::msg",
  "DriveFeedback",
  _DriveFeedback__cdr_serialize,
  _DriveFeedback__cdr_deserialize,
  _DriveFeedback__get_serialized_size,
  _DriveFeedback__max_serialized_size
};

static rosidl_message_type_support_t _DriveFeedback__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DriveFeedback,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, jackal_msgs, msg, DriveFeedback)() {
  return &_DriveFeedback__type_support;
}

#if defined(__cplusplus)
}
#endif
