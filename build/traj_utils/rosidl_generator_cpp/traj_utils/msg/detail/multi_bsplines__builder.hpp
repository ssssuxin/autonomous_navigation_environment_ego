// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__BUILDER_HPP_
#define TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__BUILDER_HPP_

#include "traj_utils/msg/detail/multi_bsplines__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace traj_utils
{

namespace msg
{

namespace builder
{

class Init_MultiBsplines_traj
{
public:
  explicit Init_MultiBsplines_traj(::traj_utils::msg::MultiBsplines & msg)
  : msg_(msg)
  {}
  ::traj_utils::msg::MultiBsplines traj(::traj_utils::msg::MultiBsplines::_traj_type arg)
  {
    msg_.traj = std::move(arg);
    return std::move(msg_);
  }

private:
  ::traj_utils::msg::MultiBsplines msg_;
};

class Init_MultiBsplines_drone_id_from
{
public:
  Init_MultiBsplines_drone_id_from()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiBsplines_traj drone_id_from(::traj_utils::msg::MultiBsplines::_drone_id_from_type arg)
  {
    msg_.drone_id_from = std::move(arg);
    return Init_MultiBsplines_traj(msg_);
  }

private:
  ::traj_utils::msg::MultiBsplines msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::traj_utils::msg::MultiBsplines>()
{
  return traj_utils::msg::builder::Init_MultiBsplines_drone_id_from();
}

}  // namespace traj_utils

#endif  // TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__BUILDER_HPP_
