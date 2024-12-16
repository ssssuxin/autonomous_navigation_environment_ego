// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice
#include "traj_utils/msg/detail/multi_bsplines__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "traj_utils/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "traj_utils/msg/detail/multi_bsplines__struct.h"
#include "traj_utils/msg/detail/multi_bsplines__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "traj_utils/msg/detail/bspline__functions.h"  // traj

// forward declare type support functions
size_t get_serialized_size_traj_utils__msg__Bspline(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_traj_utils__msg__Bspline(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, traj_utils, msg, Bspline)();


using _MultiBsplines__ros_msg_type = traj_utils__msg__MultiBsplines;

static bool _MultiBsplines__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MultiBsplines__ros_msg_type * ros_message = static_cast<const _MultiBsplines__ros_msg_type *>(untyped_ros_message);
  // Field name: drone_id_from
  {
    cdr << ros_message->drone_id_from;
  }

  // Field name: traj
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, traj_utils, msg, Bspline
      )()->data);
    size_t size = ros_message->traj.size;
    auto array_ptr = ros_message->traj.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _MultiBsplines__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MultiBsplines__ros_msg_type * ros_message = static_cast<_MultiBsplines__ros_msg_type *>(untyped_ros_message);
  // Field name: drone_id_from
  {
    cdr >> ros_message->drone_id_from;
  }

  // Field name: traj
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, traj_utils, msg, Bspline
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->traj.data) {
      traj_utils__msg__Bspline__Sequence__fini(&ros_message->traj);
    }
    if (!traj_utils__msg__Bspline__Sequence__init(&ros_message->traj, size)) {
      return "failed to create array for field 'traj'";
    }
    auto array_ptr = ros_message->traj.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_traj_utils
size_t get_serialized_size_traj_utils__msg__MultiBsplines(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MultiBsplines__ros_msg_type * ros_message = static_cast<const _MultiBsplines__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name drone_id_from
  {
    size_t item_size = sizeof(ros_message->drone_id_from);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name traj
  {
    size_t array_size = ros_message->traj.size;
    auto array_ptr = ros_message->traj.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_traj_utils__msg__Bspline(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MultiBsplines__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_traj_utils__msg__MultiBsplines(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_traj_utils
size_t max_serialized_size_traj_utils__msg__MultiBsplines(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: drone_id_from
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: traj
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_traj_utils__msg__Bspline(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _MultiBsplines__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_traj_utils__msg__MultiBsplines(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_MultiBsplines = {
  "traj_utils::msg",
  "MultiBsplines",
  _MultiBsplines__cdr_serialize,
  _MultiBsplines__cdr_deserialize,
  _MultiBsplines__get_serialized_size,
  _MultiBsplines__max_serialized_size
};

static rosidl_message_type_support_t _MultiBsplines__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MultiBsplines,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, traj_utils, msg, MultiBsplines)() {
  return &_MultiBsplines__type_support;
}

#if defined(__cplusplus)
}
#endif
