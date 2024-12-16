// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from traj_utils:msg/DataDisp.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__DATA_DISP__BUILDER_HPP_
#define TRAJ_UTILS__MSG__DETAIL__DATA_DISP__BUILDER_HPP_

#include "traj_utils/msg/detail/data_disp__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace traj_utils
{

namespace msg
{

namespace builder
{

class Init_DataDisp_e
{
public:
  explicit Init_DataDisp_e(::traj_utils::msg::DataDisp & msg)
  : msg_(msg)
  {}
  ::traj_utils::msg::DataDisp e(::traj_utils::msg::DataDisp::_e_type arg)
  {
    msg_.e = std::move(arg);
    return std::move(msg_);
  }

private:
  ::traj_utils::msg::DataDisp msg_;
};

class Init_DataDisp_d
{
public:
  explicit Init_DataDisp_d(::traj_utils::msg::DataDisp & msg)
  : msg_(msg)
  {}
  Init_DataDisp_e d(::traj_utils::msg::DataDisp::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_DataDisp_e(msg_);
  }

private:
  ::traj_utils::msg::DataDisp msg_;
};

class Init_DataDisp_c
{
public:
  explicit Init_DataDisp_c(::traj_utils::msg::DataDisp & msg)
  : msg_(msg)
  {}
  Init_DataDisp_d c(::traj_utils::msg::DataDisp::_c_type arg)
  {
    msg_.c = std::move(arg);
    return Init_DataDisp_d(msg_);
  }

private:
  ::traj_utils::msg::DataDisp msg_;
};

class Init_DataDisp_b
{
public:
  explicit Init_DataDisp_b(::traj_utils::msg::DataDisp & msg)
  : msg_(msg)
  {}
  Init_DataDisp_c b(::traj_utils::msg::DataDisp::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_DataDisp_c(msg_);
  }

private:
  ::traj_utils::msg::DataDisp msg_;
};

class Init_DataDisp_a
{
public:
  explicit Init_DataDisp_a(::traj_utils::msg::DataDisp & msg)
  : msg_(msg)
  {}
  Init_DataDisp_b a(::traj_utils::msg::DataDisp::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_DataDisp_b(msg_);
  }

private:
  ::traj_utils::msg::DataDisp msg_;
};

class Init_DataDisp_header
{
public:
  Init_DataDisp_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DataDisp_a header(::traj_utils::msg::DataDisp::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DataDisp_a(msg_);
  }

private:
  ::traj_utils::msg::DataDisp msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::traj_utils::msg::DataDisp>()
{
  return traj_utils::msg::builder::Init_DataDisp_header();
}

}  // namespace traj_utils

#endif  // TRAJ_UTILS__MSG__DETAIL__DATA_DISP__BUILDER_HPP_
