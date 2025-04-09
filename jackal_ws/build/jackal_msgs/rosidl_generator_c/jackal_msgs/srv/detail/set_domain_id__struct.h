// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jackal_msgs:srv/SetDomainId.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__SRV__DETAIL__SET_DOMAIN_ID__STRUCT_H_
#define JACKAL_MSGS__SRV__DETAIL__SET_DOMAIN_ID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetDomainId in the package jackal_msgs.
typedef struct jackal_msgs__srv__SetDomainId_Request
{
  /// ROS_DOMAIN_ID
  uint8_t domain_id;
} jackal_msgs__srv__SetDomainId_Request;

// Struct for a sequence of jackal_msgs__srv__SetDomainId_Request.
typedef struct jackal_msgs__srv__SetDomainId_Request__Sequence
{
  jackal_msgs__srv__SetDomainId_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jackal_msgs__srv__SetDomainId_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetDomainId in the package jackal_msgs.
typedef struct jackal_msgs__srv__SetDomainId_Response
{
  /// Indicate successful update of domain ID
  bool success;
  /// Error/Success message
  rosidl_runtime_c__String message;
} jackal_msgs__srv__SetDomainId_Response;

// Struct for a sequence of jackal_msgs__srv__SetDomainId_Response.
typedef struct jackal_msgs__srv__SetDomainId_Response__Sequence
{
  jackal_msgs__srv__SetDomainId_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jackal_msgs__srv__SetDomainId_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JACKAL_MSGS__SRV__DETAIL__SET_DOMAIN_ID__STRUCT_H_
