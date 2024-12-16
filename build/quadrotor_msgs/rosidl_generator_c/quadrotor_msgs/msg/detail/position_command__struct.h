// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/PositionCommand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__POSITION_COMMAND__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__POSITION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TRAJECTORY_STATUS_EMPTY'.
enum
{
  quadrotor_msgs__msg__PositionCommand__TRAJECTORY_STATUS_EMPTY = 0
};

/// Constant 'TRAJECTORY_STATUS_READY'.
enum
{
  quadrotor_msgs__msg__PositionCommand__TRAJECTORY_STATUS_READY = 1
};

/// Constant 'TRAJECTORY_STATUS_COMPLETED'.
enum
{
  quadrotor_msgs__msg__PositionCommand__TRAJECTORY_STATUS_COMPLETED = 3
};

/// Constant 'TRAJECTROY_STATUS_ABORT'.
enum
{
  quadrotor_msgs__msg__PositionCommand__TRAJECTROY_STATUS_ABORT = 4
};

/// Constant 'TRAJECTORY_STATUS_ILLEGAL_START'.
enum
{
  quadrotor_msgs__msg__PositionCommand__TRAJECTORY_STATUS_ILLEGAL_START = 5
};

/// Constant 'TRAJECTORY_STATUS_ILLEGAL_FINAL'.
enum
{
  quadrotor_msgs__msg__PositionCommand__TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
};

/// Constant 'TRAJECTORY_STATUS_IMPOSSIBLE'.
enum
{
  quadrotor_msgs__msg__PositionCommand__TRAJECTORY_STATUS_IMPOSSIBLE = 7
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.h"

// Struct defined in msg/PositionCommand in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__PositionCommand
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Vector3 acceleration;
  double yaw;
  double yaw_dot;
  double kx[3];
  double kv[3];
  uint32_t trajectory_id;
  uint8_t trajectory_flag;
} quadrotor_msgs__msg__PositionCommand;

// Struct for a sequence of quadrotor_msgs__msg__PositionCommand.
typedef struct quadrotor_msgs__msg__PositionCommand__Sequence
{
  quadrotor_msgs__msg__PositionCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__PositionCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POSITION_COMMAND__STRUCT_H_
