// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from traj_utils:msg/Bspline.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__BSPLINE__STRUCT_H_
#define TRAJ_UTILS__MSG__DETAIL__BSPLINE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'knots'
// Member 'yaw_pts'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'pos_pts'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in msg/Bspline in the package traj_utils.
typedef struct traj_utils__msg__Bspline
{
  int32_t drone_id;
  int32_t order;
  int64_t traj_id;
  builtin_interfaces__msg__Time start_time;
  rosidl_runtime_c__double__Sequence knots;
  geometry_msgs__msg__Point__Sequence pos_pts;
  rosidl_runtime_c__double__Sequence yaw_pts;
  double yaw_dt;
} traj_utils__msg__Bspline;

// Struct for a sequence of traj_utils__msg__Bspline.
typedef struct traj_utils__msg__Bspline__Sequence
{
  traj_utils__msg__Bspline * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} traj_utils__msg__Bspline__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRAJ_UTILS__MSG__DETAIL__BSPLINE__STRUCT_H_
