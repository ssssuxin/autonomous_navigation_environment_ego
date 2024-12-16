// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from traj_utils:msg/DataDisp.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__DATA_DISP__FUNCTIONS_H_
#define TRAJ_UTILS__MSG__DETAIL__DATA_DISP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "traj_utils/msg/rosidl_generator_c__visibility_control.h"

#include "traj_utils/msg/detail/data_disp__struct.h"

/// Initialize msg/DataDisp message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * traj_utils__msg__DataDisp
 * )) before or use
 * traj_utils__msg__DataDisp__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
bool
traj_utils__msg__DataDisp__init(traj_utils__msg__DataDisp * msg);

/// Finalize msg/DataDisp message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
void
traj_utils__msg__DataDisp__fini(traj_utils__msg__DataDisp * msg);

/// Create msg/DataDisp message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * traj_utils__msg__DataDisp__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
traj_utils__msg__DataDisp *
traj_utils__msg__DataDisp__create();

/// Destroy msg/DataDisp message.
/**
 * It calls
 * traj_utils__msg__DataDisp__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
void
traj_utils__msg__DataDisp__destroy(traj_utils__msg__DataDisp * msg);

/// Check for msg/DataDisp message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
bool
traj_utils__msg__DataDisp__are_equal(const traj_utils__msg__DataDisp * lhs, const traj_utils__msg__DataDisp * rhs);

/// Copy a msg/DataDisp message.
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
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
bool
traj_utils__msg__DataDisp__copy(
  const traj_utils__msg__DataDisp * input,
  traj_utils__msg__DataDisp * output);

/// Initialize array of msg/DataDisp messages.
/**
 * It allocates the memory for the number of elements and calls
 * traj_utils__msg__DataDisp__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
bool
traj_utils__msg__DataDisp__Sequence__init(traj_utils__msg__DataDisp__Sequence * array, size_t size);

/// Finalize array of msg/DataDisp messages.
/**
 * It calls
 * traj_utils__msg__DataDisp__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
void
traj_utils__msg__DataDisp__Sequence__fini(traj_utils__msg__DataDisp__Sequence * array);

/// Create array of msg/DataDisp messages.
/**
 * It allocates the memory for the array and calls
 * traj_utils__msg__DataDisp__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
traj_utils__msg__DataDisp__Sequence *
traj_utils__msg__DataDisp__Sequence__create(size_t size);

/// Destroy array of msg/DataDisp messages.
/**
 * It calls
 * traj_utils__msg__DataDisp__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
void
traj_utils__msg__DataDisp__Sequence__destroy(traj_utils__msg__DataDisp__Sequence * array);

/// Check for msg/DataDisp message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
bool
traj_utils__msg__DataDisp__Sequence__are_equal(const traj_utils__msg__DataDisp__Sequence * lhs, const traj_utils__msg__DataDisp__Sequence * rhs);

/// Copy an array of msg/DataDisp messages.
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
ROSIDL_GENERATOR_C_PUBLIC_traj_utils
bool
traj_utils__msg__DataDisp__Sequence__copy(
  const traj_utils__msg__DataDisp__Sequence * input,
  traj_utils__msg__DataDisp__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TRAJ_UTILS__MSG__DETAIL__DATA_DISP__FUNCTIONS_H_
