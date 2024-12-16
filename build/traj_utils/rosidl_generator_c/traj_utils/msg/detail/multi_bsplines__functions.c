// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice
#include "traj_utils/msg/detail/multi_bsplines__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `traj`
#include "traj_utils/msg/detail/bspline__functions.h"

bool
traj_utils__msg__MultiBsplines__init(traj_utils__msg__MultiBsplines * msg)
{
  if (!msg) {
    return false;
  }
  // drone_id_from
  // traj
  if (!traj_utils__msg__Bspline__Sequence__init(&msg->traj, 0)) {
    traj_utils__msg__MultiBsplines__fini(msg);
    return false;
  }
  return true;
}

void
traj_utils__msg__MultiBsplines__fini(traj_utils__msg__MultiBsplines * msg)
{
  if (!msg) {
    return;
  }
  // drone_id_from
  // traj
  traj_utils__msg__Bspline__Sequence__fini(&msg->traj);
}

bool
traj_utils__msg__MultiBsplines__are_equal(const traj_utils__msg__MultiBsplines * lhs, const traj_utils__msg__MultiBsplines * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // drone_id_from
  if (lhs->drone_id_from != rhs->drone_id_from) {
    return false;
  }
  // traj
  if (!traj_utils__msg__Bspline__Sequence__are_equal(
      &(lhs->traj), &(rhs->traj)))
  {
    return false;
  }
  return true;
}

bool
traj_utils__msg__MultiBsplines__copy(
  const traj_utils__msg__MultiBsplines * input,
  traj_utils__msg__MultiBsplines * output)
{
  if (!input || !output) {
    return false;
  }
  // drone_id_from
  output->drone_id_from = input->drone_id_from;
  // traj
  if (!traj_utils__msg__Bspline__Sequence__copy(
      &(input->traj), &(output->traj)))
  {
    return false;
  }
  return true;
}

traj_utils__msg__MultiBsplines *
traj_utils__msg__MultiBsplines__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  traj_utils__msg__MultiBsplines * msg = (traj_utils__msg__MultiBsplines *)allocator.allocate(sizeof(traj_utils__msg__MultiBsplines), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(traj_utils__msg__MultiBsplines));
  bool success = traj_utils__msg__MultiBsplines__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
traj_utils__msg__MultiBsplines__destroy(traj_utils__msg__MultiBsplines * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    traj_utils__msg__MultiBsplines__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
traj_utils__msg__MultiBsplines__Sequence__init(traj_utils__msg__MultiBsplines__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  traj_utils__msg__MultiBsplines * data = NULL;

  if (size) {
    data = (traj_utils__msg__MultiBsplines *)allocator.zero_allocate(size, sizeof(traj_utils__msg__MultiBsplines), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = traj_utils__msg__MultiBsplines__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        traj_utils__msg__MultiBsplines__fini(&data[i - 1]);
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
traj_utils__msg__MultiBsplines__Sequence__fini(traj_utils__msg__MultiBsplines__Sequence * array)
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
      traj_utils__msg__MultiBsplines__fini(&array->data[i]);
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

traj_utils__msg__MultiBsplines__Sequence *
traj_utils__msg__MultiBsplines__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  traj_utils__msg__MultiBsplines__Sequence * array = (traj_utils__msg__MultiBsplines__Sequence *)allocator.allocate(sizeof(traj_utils__msg__MultiBsplines__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = traj_utils__msg__MultiBsplines__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
traj_utils__msg__MultiBsplines__Sequence__destroy(traj_utils__msg__MultiBsplines__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    traj_utils__msg__MultiBsplines__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
traj_utils__msg__MultiBsplines__Sequence__are_equal(const traj_utils__msg__MultiBsplines__Sequence * lhs, const traj_utils__msg__MultiBsplines__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!traj_utils__msg__MultiBsplines__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
traj_utils__msg__MultiBsplines__Sequence__copy(
  const traj_utils__msg__MultiBsplines__Sequence * input,
  traj_utils__msg__MultiBsplines__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(traj_utils__msg__MultiBsplines);
    traj_utils__msg__MultiBsplines * data =
      (traj_utils__msg__MultiBsplines *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!traj_utils__msg__MultiBsplines__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          traj_utils__msg__MultiBsplines__fini(&data[i]);
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
    if (!traj_utils__msg__MultiBsplines__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
