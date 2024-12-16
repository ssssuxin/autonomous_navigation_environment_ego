// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "traj_utils/msg/detail/multi_bsplines__rosidl_typesupport_introspection_c.h"
#include "traj_utils/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "traj_utils/msg/detail/multi_bsplines__functions.h"
#include "traj_utils/msg/detail/multi_bsplines__struct.h"


// Include directives for member types
// Member `traj`
#include "traj_utils/msg/bspline.h"
// Member `traj`
#include "traj_utils/msg/detail/bspline__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  traj_utils__msg__MultiBsplines__init(message_memory);
}

void MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_fini_function(void * message_memory)
{
  traj_utils__msg__MultiBsplines__fini(message_memory);
}

size_t MultiBsplines__rosidl_typesupport_introspection_c__size_function__Bspline__traj(
  const void * untyped_member)
{
  const traj_utils__msg__Bspline__Sequence * member =
    (const traj_utils__msg__Bspline__Sequence *)(untyped_member);
  return member->size;
}

const void * MultiBsplines__rosidl_typesupport_introspection_c__get_const_function__Bspline__traj(
  const void * untyped_member, size_t index)
{
  const traj_utils__msg__Bspline__Sequence * member =
    (const traj_utils__msg__Bspline__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MultiBsplines__rosidl_typesupport_introspection_c__get_function__Bspline__traj(
  void * untyped_member, size_t index)
{
  traj_utils__msg__Bspline__Sequence * member =
    (traj_utils__msg__Bspline__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MultiBsplines__rosidl_typesupport_introspection_c__resize_function__Bspline__traj(
  void * untyped_member, size_t size)
{
  traj_utils__msg__Bspline__Sequence * member =
    (traj_utils__msg__Bspline__Sequence *)(untyped_member);
  traj_utils__msg__Bspline__Sequence__fini(member);
  return traj_utils__msg__Bspline__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_message_member_array[2] = {
  {
    "drone_id_from",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__MultiBsplines, drone_id_from),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "traj",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils__msg__MultiBsplines, traj),  // bytes offset in struct
    NULL,  // default value
    MultiBsplines__rosidl_typesupport_introspection_c__size_function__Bspline__traj,  // size() function pointer
    MultiBsplines__rosidl_typesupport_introspection_c__get_const_function__Bspline__traj,  // get_const(index) function pointer
    MultiBsplines__rosidl_typesupport_introspection_c__get_function__Bspline__traj,  // get(index) function pointer
    MultiBsplines__rosidl_typesupport_introspection_c__resize_function__Bspline__traj  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_message_members = {
  "traj_utils__msg",  // message namespace
  "MultiBsplines",  // message name
  2,  // number of fields
  sizeof(traj_utils__msg__MultiBsplines),
  MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_message_member_array,  // message members
  MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_init_function,  // function to initialize message memory (memory has to be allocated)
  MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_message_type_support_handle = {
  0,
  &MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_traj_utils
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, traj_utils, msg, MultiBsplines)() {
  MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, traj_utils, msg, Bspline)();
  if (!MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_message_type_support_handle.typesupport_identifier) {
    MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MultiBsplines__rosidl_typesupport_introspection_c__MultiBsplines_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
