// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jackal_msgs:msg/Power.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__POWER__STRUCT_H_
#define JACKAL_MSGS__MSG__DETAIL__POWER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NOT_APPLICABLE'.
/**
  * AC Power
 */
enum
{
  jackal_msgs__msg__Power__NOT_APPLICABLE = -1
};

/// Constant 'JACKAL_MEASURED_BATTERY'.
/**
  * Voltage rails, in volts
  * Averaged over the message period
  * Jackal
 */
enum
{
  jackal_msgs__msg__Power__JACKAL_MEASURED_BATTERY = 0
};

/// Constant 'JACKAL_MEASURED_5V'.
enum
{
  jackal_msgs__msg__Power__JACKAL_MEASURED_5V = 1
};

/// Constant 'JACKAL_MEASURED_12V'.
enum
{
  jackal_msgs__msg__Power__JACKAL_MEASURED_12V = 2
};

/// Constant 'JACKAL_TOTAL_CURRENT'.
/**
  * Current senses available on platform, in amps.
  * Averaged over the message period
  * Jackal
 */
enum
{
  jackal_msgs__msg__Power__JACKAL_TOTAL_CURRENT = 0
};

/// Constant 'JACKAL_COMPUTER_CURRENT'.
enum
{
  jackal_msgs__msg__Power__JACKAL_COMPUTER_CURRENT = 1
};

/// Constant 'JACKAL_DRIVE_CURRENT'.
enum
{
  jackal_msgs__msg__Power__JACKAL_DRIVE_CURRENT = 2
};

/// Constant 'JACKAL_USER_CURRENT'.
enum
{
  jackal_msgs__msg__Power__JACKAL_USER_CURRENT = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'measured_voltages'
// Member 'measured_currents'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Power in the package jackal_msgs.
/**
  * Robot Power readings
 */
typedef struct jackal_msgs__msg__Power
{
  std_msgs__msg__Header header;
  /// Indicates if AC power is connected.
  int8_t shore_power_connected;
  /// Indicates if battery is connected.
  int8_t battery_connected;
  /// Indicates if the 12V user power is good.
  int8_t power_12v_user_nominal;
  /// Indicates if charging is complete.
  int8_t charging_complete;
  rosidl_runtime_c__float__Sequence measured_voltages;
  rosidl_runtime_c__float__Sequence measured_currents;
} jackal_msgs__msg__Power;

// Struct for a sequence of jackal_msgs__msg__Power.
typedef struct jackal_msgs__msg__Power__Sequence
{
  jackal_msgs__msg__Power * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jackal_msgs__msg__Power__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JACKAL_MSGS__MSG__DETAIL__POWER__STRUCT_H_
