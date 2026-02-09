// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from exia_msgs:msg/NavigationGoal.idl
// generated code does not contain a copyright notice

#ifndef EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__FUNCTIONS_H_
#define EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "exia_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "exia_msgs/msg/detail/navigation_goal__struct.h"

/// Initialize msg/NavigationGoal message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * exia_msgs__msg__NavigationGoal
 * )) before or use
 * exia_msgs__msg__NavigationGoal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
bool
exia_msgs__msg__NavigationGoal__init(exia_msgs__msg__NavigationGoal * msg);

/// Finalize msg/NavigationGoal message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
void
exia_msgs__msg__NavigationGoal__fini(exia_msgs__msg__NavigationGoal * msg);

/// Create msg/NavigationGoal message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * exia_msgs__msg__NavigationGoal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
exia_msgs__msg__NavigationGoal *
exia_msgs__msg__NavigationGoal__create();

/// Destroy msg/NavigationGoal message.
/**
 * It calls
 * exia_msgs__msg__NavigationGoal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
void
exia_msgs__msg__NavigationGoal__destroy(exia_msgs__msg__NavigationGoal * msg);

/// Check for msg/NavigationGoal message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
bool
exia_msgs__msg__NavigationGoal__are_equal(const exia_msgs__msg__NavigationGoal * lhs, const exia_msgs__msg__NavigationGoal * rhs);

/// Copy a msg/NavigationGoal message.
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
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
bool
exia_msgs__msg__NavigationGoal__copy(
  const exia_msgs__msg__NavigationGoal * input,
  exia_msgs__msg__NavigationGoal * output);

/// Initialize array of msg/NavigationGoal messages.
/**
 * It allocates the memory for the number of elements and calls
 * exia_msgs__msg__NavigationGoal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
bool
exia_msgs__msg__NavigationGoal__Sequence__init(exia_msgs__msg__NavigationGoal__Sequence * array, size_t size);

/// Finalize array of msg/NavigationGoal messages.
/**
 * It calls
 * exia_msgs__msg__NavigationGoal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
void
exia_msgs__msg__NavigationGoal__Sequence__fini(exia_msgs__msg__NavigationGoal__Sequence * array);

/// Create array of msg/NavigationGoal messages.
/**
 * It allocates the memory for the array and calls
 * exia_msgs__msg__NavigationGoal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
exia_msgs__msg__NavigationGoal__Sequence *
exia_msgs__msg__NavigationGoal__Sequence__create(size_t size);

/// Destroy array of msg/NavigationGoal messages.
/**
 * It calls
 * exia_msgs__msg__NavigationGoal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
void
exia_msgs__msg__NavigationGoal__Sequence__destroy(exia_msgs__msg__NavigationGoal__Sequence * array);

/// Check for msg/NavigationGoal message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
bool
exia_msgs__msg__NavigationGoal__Sequence__are_equal(const exia_msgs__msg__NavigationGoal__Sequence * lhs, const exia_msgs__msg__NavigationGoal__Sequence * rhs);

/// Copy an array of msg/NavigationGoal messages.
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
ROSIDL_GENERATOR_C_PUBLIC_exia_msgs
bool
exia_msgs__msg__NavigationGoal__Sequence__copy(
  const exia_msgs__msg__NavigationGoal__Sequence * input,
  exia_msgs__msg__NavigationGoal__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__FUNCTIONS_H_
