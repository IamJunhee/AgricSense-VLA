// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from jackal_msgs:msg/DriveFeedback.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__FUNCTIONS_H_
#define JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "jackal_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "jackal_msgs/msg/detail/drive_feedback__struct.h"

/// Initialize msg/DriveFeedback message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jackal_msgs__msg__DriveFeedback
 * )) before or use
 * jackal_msgs__msg__DriveFeedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
bool
jackal_msgs__msg__DriveFeedback__init(jackal_msgs__msg__DriveFeedback * msg);

/// Finalize msg/DriveFeedback message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
void
jackal_msgs__msg__DriveFeedback__fini(jackal_msgs__msg__DriveFeedback * msg);

/// Create msg/DriveFeedback message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jackal_msgs__msg__DriveFeedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
jackal_msgs__msg__DriveFeedback *
jackal_msgs__msg__DriveFeedback__create();

/// Destroy msg/DriveFeedback message.
/**
 * It calls
 * jackal_msgs__msg__DriveFeedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
void
jackal_msgs__msg__DriveFeedback__destroy(jackal_msgs__msg__DriveFeedback * msg);

/// Check for msg/DriveFeedback message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
bool
jackal_msgs__msg__DriveFeedback__are_equal(const jackal_msgs__msg__DriveFeedback * lhs, const jackal_msgs__msg__DriveFeedback * rhs);

/// Copy a msg/DriveFeedback message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
bool
jackal_msgs__msg__DriveFeedback__copy(
  const jackal_msgs__msg__DriveFeedback * input,
  jackal_msgs__msg__DriveFeedback * output);

/// Initialize array of msg/DriveFeedback messages.
/**
 * It allocates the memory for the number of elements and calls
 * jackal_msgs__msg__DriveFeedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
bool
jackal_msgs__msg__DriveFeedback__Sequence__init(jackal_msgs__msg__DriveFeedback__Sequence * array, size_t size);

/// Finalize array of msg/DriveFeedback messages.
/**
 * It calls
 * jackal_msgs__msg__DriveFeedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
void
jackal_msgs__msg__DriveFeedback__Sequence__fini(jackal_msgs__msg__DriveFeedback__Sequence * array);

/// Create array of msg/DriveFeedback messages.
/**
 * It allocates the memory for the array and calls
 * jackal_msgs__msg__DriveFeedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
jackal_msgs__msg__DriveFeedback__Sequence *
jackal_msgs__msg__DriveFeedback__Sequence__create(size_t size);

/// Destroy array of msg/DriveFeedback messages.
/**
 * It calls
 * jackal_msgs__msg__DriveFeedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
void
jackal_msgs__msg__DriveFeedback__Sequence__destroy(jackal_msgs__msg__DriveFeedback__Sequence * array);

/// Check for msg/DriveFeedback message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
bool
jackal_msgs__msg__DriveFeedback__Sequence__are_equal(const jackal_msgs__msg__DriveFeedback__Sequence * lhs, const jackal_msgs__msg__DriveFeedback__Sequence * rhs);

/// Copy an array of msg/DriveFeedback messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_jackal_msgs
bool
jackal_msgs__msg__DriveFeedback__Sequence__copy(
  const jackal_msgs__msg__DriveFeedback__Sequence * input,
  jackal_msgs__msg__DriveFeedback__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__FUNCTIONS_H_
