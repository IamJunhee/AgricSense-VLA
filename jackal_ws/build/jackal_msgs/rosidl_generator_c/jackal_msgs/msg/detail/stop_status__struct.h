// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jackal_msgs:msg/StopStatus.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__STOP_STATUS__STRUCT_H_
#define JACKAL_MSGS__MSG__DETAIL__STOP_STATUS__STRUCT_H_

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

/// Struct defined in msg/StopStatus in the package jackal_msgs.
/**
  * Additional Stop information
 */
typedef struct jackal_msgs__msg__StopStatus
{
  std_msgs__msg__Header header;
  /// True indicates the stop loop is operational (ie. it is powered).
  bool stop_power_status;
  /// True indicates a external stop has been plugged in.
  bool external_stop_present;
} jackal_msgs__msg__StopStatus;

// Struct for a sequence of jackal_msgs__msg__StopStatus.
typedef struct jackal_msgs__msg__StopStatus__Sequence
{
  jackal_msgs__msg__StopStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jackal_msgs__msg__StopStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JACKAL_MSGS__MSG__DETAIL__STOP_STATUS__STRUCT_H_
