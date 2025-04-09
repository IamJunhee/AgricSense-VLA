// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jackal_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
#define JACKAL_MSGS__MSG__DETAIL__STATUS__STRUCT_H_

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
// Member 'hardware_id'
// Member 'firmware_commit'
#include "rosidl_runtime_c/string.h"
// Member 'mcu_uptime'
// Member 'connection_uptime'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in msg/Status in the package jackal_msgs.
/**
  * This message represents lower-frequency status updates
  * Default publish frequency is 1Hz.
 */
typedef struct jackal_msgs__msg__Status
{
  std_msgs__msg__Header header;
  /// Robot Hardware ID
  rosidl_runtime_c__String hardware_id;
  /// Firmware commit hash
  rosidl_runtime_c__String firmware_commit;
  /// Times since MCU power-on.
  builtin_interfaces__msg__Duration mcu_uptime;
  builtin_interfaces__msg__Duration connection_uptime;
  /// Temperature of MCU's PCB in Celsius.
  float pcb_temperature;
  float mcu_temperature;
} jackal_msgs__msg__Status;

// Struct for a sequence of jackal_msgs__msg__Status.
typedef struct jackal_msgs__msg__Status__Sequence
{
  jackal_msgs__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jackal_msgs__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JACKAL_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
