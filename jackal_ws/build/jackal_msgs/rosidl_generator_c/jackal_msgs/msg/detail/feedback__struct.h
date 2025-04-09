// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jackal_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_
#define JACKAL_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'drivers'
#include "jackal_msgs/msg/detail/drive_feedback__struct.h"

/// Struct defined in msg/Feedback in the package jackal_msgs.
/**
  * This message represents high-frequency feedback from the MCU,
  * as necessary to support closed-loop control and thermal monitoring.
  * Default publish frequency is 50Hz.
 */
typedef struct jackal_msgs__msg__Feedback
{
  std_msgs__msg__Header header;
  jackal_msgs__msg__DriveFeedback drivers[2];
  /// Commanded control mode, use the TYPE_ constants from jackal_msgs/Drive.
  int8_t commanded_mode;
  /// Actual control mode. This may differ from the commanded in cases where
  /// the motor enable is off, the motors are in over-current, etc.
  int8_t actual_mode;
} jackal_msgs__msg__Feedback;

// Struct for a sequence of jackal_msgs__msg__Feedback.
typedef struct jackal_msgs__msg__Feedback__Sequence
{
  jackal_msgs__msg__Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jackal_msgs__msg__Feedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JACKAL_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_
