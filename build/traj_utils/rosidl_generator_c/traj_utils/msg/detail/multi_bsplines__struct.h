// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__STRUCT_H_
#define TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'traj'
#include "traj_utils/msg/detail/bspline__struct.h"

// Struct defined in msg/MultiBsplines in the package traj_utils.
typedef struct traj_utils__msg__MultiBsplines
{
  int32_t drone_id_from;
  traj_utils__msg__Bspline__Sequence traj;
} traj_utils__msg__MultiBsplines;

// Struct for a sequence of traj_utils__msg__MultiBsplines.
typedef struct traj_utils__msg__MultiBsplines__Sequence
{
  traj_utils__msg__MultiBsplines * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} traj_utils__msg__MultiBsplines__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__STRUCT_H_
