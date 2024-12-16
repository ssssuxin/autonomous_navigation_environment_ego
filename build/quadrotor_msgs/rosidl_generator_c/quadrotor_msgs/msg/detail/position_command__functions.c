// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/PositionCommand.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/position_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
// Member `acceleration`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
quadrotor_msgs__msg__PositionCommand__init(quadrotor_msgs__msg__PositionCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quadrotor_msgs__msg__PositionCommand__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    quadrotor_msgs__msg__PositionCommand__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    quadrotor_msgs__msg__PositionCommand__fini(msg);
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration)) {
    quadrotor_msgs__msg__PositionCommand__fini(msg);
    return false;
  }
  // yaw
  // yaw_dot
  // kx
  // kv
  // trajectory_id
  // trajectory_flag
  return true;
}

void
quadrotor_msgs__msg__PositionCommand__fini(quadrotor_msgs__msg__PositionCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // acceleration
  geometry_msgs__msg__Vector3__fini(&msg->acceleration);
  // yaw
  // yaw_dot
  // kx
  // kv
  // trajectory_id
  // trajectory_flag
}

bool
quadrotor_msgs__msg__PositionCommand__are_equal(const quadrotor_msgs__msg__PositionCommand * lhs, const quadrotor_msgs__msg__PositionCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // yaw_dot
  if (lhs->yaw_dot != rhs->yaw_dot) {
    return false;
  }
  // kx
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->kx[i] != rhs->kx[i]) {
      return false;
    }
  }
  // kv
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->kv[i] != rhs->kv[i]) {
      return false;
    }
  }
  // trajectory_id
  if (lhs->trajectory_id != rhs->trajectory_id) {
    return false;
  }
  // trajectory_flag
  if (lhs->trajectory_flag != rhs->trajectory_flag) {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__PositionCommand__copy(
  const quadrotor_msgs__msg__PositionCommand * input,
  quadrotor_msgs__msg__PositionCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // yaw_dot
  output->yaw_dot = input->yaw_dot;
  // kx
  for (size_t i = 0; i < 3; ++i) {
    output->kx[i] = input->kx[i];
  }
  // kv
  for (size_t i = 0; i < 3; ++i) {
    output->kv[i] = input->kv[i];
  }
  // trajectory_id
  output->trajectory_id = input->trajectory_id;
  // trajectory_flag
  output->trajectory_flag = input->trajectory_flag;
  return true;
}

quadrotor_msgs__msg__PositionCommand *
quadrotor_msgs__msg__PositionCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__PositionCommand * msg = (quadrotor_msgs__msg__PositionCommand *)allocator.allocate(sizeof(quadrotor_msgs__msg__PositionCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__PositionCommand));
  bool success = quadrotor_msgs__msg__PositionCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__PositionCommand__destroy(quadrotor_msgs__msg__PositionCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__PositionCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__PositionCommand__Sequence__init(quadrotor_msgs__msg__PositionCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__PositionCommand * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__PositionCommand *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__PositionCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__PositionCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__PositionCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
quadrotor_msgs__msg__PositionCommand__Sequence__fini(quadrotor_msgs__msg__PositionCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      quadrotor_msgs__msg__PositionCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

quadrotor_msgs__msg__PositionCommand__Sequence *
quadrotor_msgs__msg__PositionCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__PositionCommand__Sequence * array = (quadrotor_msgs__msg__PositionCommand__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__PositionCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__PositionCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__PositionCommand__Sequence__destroy(quadrotor_msgs__msg__PositionCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__PositionCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__PositionCommand__Sequence__are_equal(const quadrotor_msgs__msg__PositionCommand__Sequence * lhs, const quadrotor_msgs__msg__PositionCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__PositionCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__PositionCommand__Sequence__copy(
  const quadrotor_msgs__msg__PositionCommand__Sequence * input,
  quadrotor_msgs__msg__PositionCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__PositionCommand);
    quadrotor_msgs__msg__PositionCommand * data =
      (quadrotor_msgs__msg__PositionCommand *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__PositionCommand__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__PositionCommand__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__PositionCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
