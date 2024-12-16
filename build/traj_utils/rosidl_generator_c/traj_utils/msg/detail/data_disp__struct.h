// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from traj_utils:msg/DataDisp.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__DATA_DISP__STRUCT_H_
#define TRAJ_UTILS__MSG__DETAIL__DATA_DISP__STRUCT_H_

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

// Struct defined in msg/DataDisp in the package traj_utils.
typedef struct traj_utils__msg__DataDisp
{
  std_msgs__msg__Header header;
  double a;
  double b;
  double c;
  double d;
  double e;
} traj_utils__msg__DataDisp;

// Struct for a sequence of traj_utils__msg__DataDisp.
typedef struct traj_utils__msg__DataDisp__Sequence
{
  traj_utils__msg__DataDisp * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} traj_utils__msg__DataDisp__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAJ_UTILS__MSG__DETAIL__DATA_DISP__STRUCT_H_
