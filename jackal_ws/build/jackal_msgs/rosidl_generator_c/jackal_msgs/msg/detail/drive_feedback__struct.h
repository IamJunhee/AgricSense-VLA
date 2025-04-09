// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jackal_msgs:msg/DriveFeedback.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__STRUCT_H_
#define JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DriveFeedback in the package jackal_msgs.
/**
  * This message represents feedback data from a single drive unit (driver + motor).
 */
typedef struct jackal_msgs__msg__DriveFeedback
{
  /// Current flowing from battery into the MOSFET bridge.
  float current;
  /// Instantaneous duty cycle of MOSFET bridge.
  float duty_cycle;
  /// Temperatures measured in the MOSFET bridge and on the motor casing, in deg C.
  float bridge_temperature;
  float motor_temperature;
  /// Encoder data
  /// rad/s
  float measured_velocity;
  /// rad
  float measured_travel;
  /// True if the underlying driver chip reports a fault condition.
  bool driver_fault;
} jackal_msgs__msg__DriveFeedback;

// Struct for a sequence of jackal_msgs__msg__DriveFeedback.
typedef struct jackal_msgs__msg__DriveFeedback__Sequence
{
  jackal_msgs__msg__DriveFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jackal_msgs__msg__DriveFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__STRUCT_H_
