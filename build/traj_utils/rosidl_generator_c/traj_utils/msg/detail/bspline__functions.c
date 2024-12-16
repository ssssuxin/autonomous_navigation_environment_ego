// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from traj_utils:msg/Bspline.idl
// generated code does not contain a copyright notice
#include "traj_utils/msg/detail/bspline__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `start_time`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `knots`
// Member `yaw_pts`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `pos_pts`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
traj_utils__msg__Bspline__init(traj_utils__msg__Bspline * msg)
{
  if (!msg) {
    return false;
  }
  // drone_id
  // order
  // traj_id
  // start_time
  if (!builtin_interfaces__msg__Time__init(&msg->start_time)) {
    traj_utils__msg__Bspline__fini(msg);
    return false;
  }
  // knots
  if (!rosidl_runtime_c__double__Sequence__init(&msg->knots, 0)) {
    traj_utils__msg__Bspline__fini(msg);
    return false;
  }
  // pos_pts
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->pos_pts, 0)) {
    traj_utils__msg__Bspline__fini(msg);
    return false;
  }
  // yaw_pts
  if (!rosidl_runtime_c__double__Sequence__init(&msg->yaw_pts, 0)) {
    traj_utils__msg__Bspline__fini(msg);
    return false;
  }
  // yaw_dt
  return true;
}

void
traj_utils__msg__Bspline__fini(traj_utils__msg__Bspline * msg)
{
  if (!msg) {
    return;
  }
  // drone_id
  // order
  // traj_id
  // start_time
  builtin_interfaces__msg__Time__fini(&msg->start_time);
  // knots
  rosidl_runtime_c__double__Sequence__fini(&msg->knots);
  // pos_pts
  geometry_msgs__msg__Point__Sequence__fini(&msg->pos_pts);
  // yaw_pts
  rosidl_runtime_c__double__Sequence__fini(&msg->yaw_pts);
  // yaw_dt
}

bool
traj_utils__msg__Bspline__are_equal(const traj_utils__msg__Bspline * lhs, const traj_utils__msg__Bspline * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // drone_id
  if (lhs->drone_id != rhs->drone_id) {
    return false;
  }
  // order
  if (lhs->order != rhs->order) {
    return false;
  }
  // traj_id
  if (lhs->traj_id != rhs->traj_id) {
    return false;
  }
  // start_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->start_time), &(rhs->start_time)))
  {
    return false;
  }
  // knots
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->knots), &(rhs->knots)))
  {
    return false;
  }
  // pos_pts
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->pos_pts), &(rhs->pos_pts)))
  {
    return false;
  }
  // yaw_pts
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->yaw_pts), &(rhs->yaw_pts)))
  {
    return false;
  }
  // yaw_dt
  if (lhs->yaw_dt != rhs->yaw_dt) {
    return false;
  }
  return true;
}

bool
traj_utils__msg__Bspline__copy(
  const traj_utils__msg__Bspline * input,
  traj_utils__msg__Bspline * output)
{
  if (!input || !output) {
    return false;
  }
  // drone_id
  output->drone_id = input->drone_id;
  // order
  output->order = input->order;
  // traj_id
  output->traj_id = input->traj_id;
  // start_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->start_time), &(output->start_time)))
  {
    return false;
  }
  // knots
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->knots), &(output->knots)))
  {
    return false;
  }
  // pos_pts
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->pos_pts), &(output->pos_pts)))
  {
    return false;
  }
  // yaw_pts
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->yaw_pts), &(output->yaw_pts)))
  {
    return false;
  }
  // yaw_dt
  output->yaw_dt = input->yaw_dt;
  return true;
}

traj_utils__msg__Bspline *
traj_utils__msg__Bspline__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  traj_utils__msg__Bspline * msg = (traj_utils__msg__Bspline *)allocator.allocate(sizeof(traj_utils__msg__Bspline), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(traj_utils__msg__Bspline));
  bool success = traj_utils__msg__Bspline__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
traj_utils__msg__Bspline__destroy(traj_utils__msg__Bspline * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    traj_utils__msg__Bspline__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
traj_utils__msg__Bspline__Sequence__init(traj_utils__msg__Bspline__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  traj_utils__msg__Bspline * data = NULL;

  if (size) {
    data = (traj_utils__msg__Bspline *)allocator.zero_allocate(size, sizeof(traj_utils__msg__Bspline), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = traj_utils__msg__Bspline__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        traj_utils__msg__Bspline__fini(&data[i - 1]);
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
traj_utils__msg__Bspline__Sequence__fini(traj_utils__msg__Bspline__Sequence * array)
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
      traj_utils__msg__Bspline__fini(&array->data[i]);
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

traj_utils__msg__Bspline__Sequence *
traj_utils__msg__Bspline__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  traj_utils__msg__Bspline__Sequence * array = (traj_utils__msg__Bspline__Sequence *)allocator.allocate(sizeof(traj_utils__msg__Bspline__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = traj_utils__msg__Bspline__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
traj_utils__msg__Bspline__Sequence__destroy(traj_utils__msg__Bspline__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    traj_utils__msg__Bspline__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
traj_utils__msg__Bspline__Sequence__are_equal(const traj_utils__msg__Bspline__Sequence * lhs, const traj_utils__msg__Bspline__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!traj_utils__msg__Bspline__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
traj_utils__msg__Bspline__Sequence__copy(
  const traj_utils__msg__Bspline__Sequence * input,
  traj_utils__msg__Bspline__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(traj_utils__msg__Bspline);
    traj_utils__msg__Bspline * data =
      (traj_utils__msg__Bspline *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!traj_utils__msg__Bspline__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          traj_utils__msg__Bspline__fini(&data[i]);
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
    if (!traj_utils__msg__Bspline__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
