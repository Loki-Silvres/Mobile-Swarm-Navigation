// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from control_msgs:msg/DynamicInterfaceGroupValues.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__FUNCTIONS_H_
#define CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "control_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "control_msgs/msg/detail/dynamic_interface_group_values__struct.h"

/// Initialize msg/DynamicInterfaceGroupValues message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * control_msgs__msg__DynamicInterfaceGroupValues
 * )) before or use
 * control_msgs__msg__DynamicInterfaceGroupValues__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
bool
control_msgs__msg__DynamicInterfaceGroupValues__init(control_msgs__msg__DynamicInterfaceGroupValues * msg);

/// Finalize msg/DynamicInterfaceGroupValues message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
void
control_msgs__msg__DynamicInterfaceGroupValues__fini(control_msgs__msg__DynamicInterfaceGroupValues * msg);

/// Create msg/DynamicInterfaceGroupValues message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * control_msgs__msg__DynamicInterfaceGroupValues__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
control_msgs__msg__DynamicInterfaceGroupValues *
control_msgs__msg__DynamicInterfaceGroupValues__create();

/// Destroy msg/DynamicInterfaceGroupValues message.
/**
 * It calls
 * control_msgs__msg__DynamicInterfaceGroupValues__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
void
control_msgs__msg__DynamicInterfaceGroupValues__destroy(control_msgs__msg__DynamicInterfaceGroupValues * msg);

/// Check for msg/DynamicInterfaceGroupValues message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
bool
control_msgs__msg__DynamicInterfaceGroupValues__are_equal(const control_msgs__msg__DynamicInterfaceGroupValues * lhs, const control_msgs__msg__DynamicInterfaceGroupValues * rhs);

/// Copy a msg/DynamicInterfaceGroupValues message.
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
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
bool
control_msgs__msg__DynamicInterfaceGroupValues__copy(
  const control_msgs__msg__DynamicInterfaceGroupValues * input,
  control_msgs__msg__DynamicInterfaceGroupValues * output);

/// Initialize array of msg/DynamicInterfaceGroupValues messages.
/**
 * It allocates the memory for the number of elements and calls
 * control_msgs__msg__DynamicInterfaceGroupValues__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
bool
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__init(control_msgs__msg__DynamicInterfaceGroupValues__Sequence * array, size_t size);

/// Finalize array of msg/DynamicInterfaceGroupValues messages.
/**
 * It calls
 * control_msgs__msg__DynamicInterfaceGroupValues__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
void
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__fini(control_msgs__msg__DynamicInterfaceGroupValues__Sequence * array);

/// Create array of msg/DynamicInterfaceGroupValues messages.
/**
 * It allocates the memory for the array and calls
 * control_msgs__msg__DynamicInterfaceGroupValues__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
control_msgs__msg__DynamicInterfaceGroupValues__Sequence *
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__create(size_t size);

/// Destroy array of msg/DynamicInterfaceGroupValues messages.
/**
 * It calls
 * control_msgs__msg__DynamicInterfaceGroupValues__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
void
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__destroy(control_msgs__msg__DynamicInterfaceGroupValues__Sequence * array);

/// Check for msg/DynamicInterfaceGroupValues message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
bool
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__are_equal(const control_msgs__msg__DynamicInterfaceGroupValues__Sequence * lhs, const control_msgs__msg__DynamicInterfaceGroupValues__Sequence * rhs);

/// Copy an array of msg/DynamicInterfaceGroupValues messages.
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
ROSIDL_GENERATOR_C_PUBLIC_control_msgs
bool
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__copy(
  const control_msgs__msg__DynamicInterfaceGroupValues__Sequence * input,
  control_msgs__msg__DynamicInterfaceGroupValues__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__FUNCTIONS_H_
