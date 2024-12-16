// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "traj_utils/msg/detail/multi_bsplines__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace traj_utils
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MultiBsplines_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) traj_utils::msg::MultiBsplines(_init);
}

void MultiBsplines_fini_function(void * message_memory)
{
  auto typed_message = static_cast<traj_utils::msg::MultiBsplines *>(message_memory);
  typed_message->~MultiBsplines();
}

size_t size_function__MultiBsplines__traj(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<traj_utils::msg::Bspline> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiBsplines__traj(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<traj_utils::msg::Bspline> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiBsplines__traj(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<traj_utils::msg::Bspline> *>(untyped_member);
  return &member[index];
}

void resize_function__MultiBsplines__traj(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<traj_utils::msg::Bspline> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MultiBsplines_message_member_array[2] = {
  {
    "drone_id_from",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils::msg::MultiBsplines, drone_id_from),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "traj",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<traj_utils::msg::Bspline>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(traj_utils::msg::MultiBsplines, traj),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiBsplines__traj,  // size() function pointer
    get_const_function__MultiBsplines__traj,  // get_const(index) function pointer
    get_function__MultiBsplines__traj,  // get(index) function pointer
    resize_function__MultiBsplines__traj  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MultiBsplines_message_members = {
  "traj_utils::msg",  // message namespace
  "MultiBsplines",  // message name
  2,  // number of fields
  sizeof(traj_utils::msg::MultiBsplines),
  MultiBsplines_message_member_array,  // message members
  MultiBsplines_init_function,  // function to initialize message memory (memory has to be allocated)
  MultiBsplines_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MultiBsplines_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MultiBsplines_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace traj_utils


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<traj_utils::msg::MultiBsplines>()
{
  return &::traj_utils::msg::rosidl_typesupport_introspection_cpp::MultiBsplines_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, traj_utils, msg, MultiBsplines)() {
  return &::traj_utils::msg::rosidl_typesupport_introspection_cpp::MultiBsplines_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
