// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice
#include "traj_utils/msg/detail/multi_bsplines__rosidl_typesupport_fastrtps_cpp.hpp"
#include "traj_utils/msg/detail/multi_bsplines__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace traj_utils
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const traj_utils::msg::Bspline &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  traj_utils::msg::Bspline &);
size_t get_serialized_size(
  const traj_utils::msg::Bspline &,
  size_t current_alignment);
size_t
max_serialized_size_Bspline(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace traj_utils


namespace traj_utils
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traj_utils
cdr_serialize(
  const traj_utils::msg::MultiBsplines & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: drone_id_from
  cdr << ros_message.drone_id_from;
  // Member: traj
  {
    size_t size = ros_message.traj.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      traj_utils::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.traj[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traj_utils
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  traj_utils::msg::MultiBsplines & ros_message)
{
  // Member: drone_id_from
  cdr >> ros_message.drone_id_from;

  // Member: traj
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.traj.resize(size);
    for (size_t i = 0; i < size; i++) {
      traj_utils::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.traj[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traj_utils
get_serialized_size(
  const traj_utils::msg::MultiBsplines & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: drone_id_from
  {
    size_t item_size = sizeof(ros_message.drone_id_from);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: traj
  {
    size_t array_size = ros_message.traj.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        traj_utils::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.traj[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_traj_utils
max_serialized_size_MultiBsplines(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: drone_id_from
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: traj
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        traj_utils::msg::typesupport_fastrtps_cpp::max_serialized_size_Bspline(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _MultiBsplines__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const traj_utils::msg::MultiBsplines *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MultiBsplines__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<traj_utils::msg::MultiBsplines *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MultiBsplines__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const traj_utils::msg::MultiBsplines *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MultiBsplines__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_MultiBsplines(full_bounded, 0);
}

static message_type_support_callbacks_t _MultiBsplines__callbacks = {
  "traj_utils::msg",
  "MultiBsplines",
  _MultiBsplines__cdr_serialize,
  _MultiBsplines__cdr_deserialize,
  _MultiBsplines__get_serialized_size,
  _MultiBsplines__max_serialized_size
};

static rosidl_message_type_support_t _MultiBsplines__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MultiBsplines__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace traj_utils

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_traj_utils
const rosidl_message_type_support_t *
get_message_type_support_handle<traj_utils::msg::MultiBsplines>()
{
  return &traj_utils::msg::typesupport_fastrtps_cpp::_MultiBsplines__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, traj_utils, msg, MultiBsplines)() {
  return &traj_utils::msg::typesupport_fastrtps_cpp::_MultiBsplines__handle;
}

#ifdef __cplusplus
}
#endif
