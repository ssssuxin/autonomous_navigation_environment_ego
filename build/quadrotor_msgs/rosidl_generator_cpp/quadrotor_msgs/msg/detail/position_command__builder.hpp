// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quadrotor_msgs:msg/PositionCommand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__POSITION_COMMAND__BUILDER_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__POSITION_COMMAND__BUILDER_HPP_

#include "quadrotor_msgs/msg/detail/position_command__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace quadrotor_msgs
{

namespace msg
{

namespace builder
{

class Init_PositionCommand_trajectory_flag
{
public:
  explicit Init_PositionCommand_trajectory_flag(::quadrotor_msgs::msg::PositionCommand & msg)
  : msg_(msg)
  {}
  ::quadrotor_msgs::msg::PositionCommand trajectory_flag(::quadrotor_msgs::msg::PositionCommand::_trajectory_flag_type arg)
  {
    msg_.trajectory_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

class Init_PositionCommand_trajectory_id
{
public:
  explicit Init_PositionCommand_trajectory_id(::quadrotor_msgs::msg::PositionCommand & msg)
  : msg_(msg)
  {}
  Init_PositionCommand_trajectory_flag trajectory_id(::quadrotor_msgs::msg::PositionCommand::_trajectory_id_type arg)
  {
    msg_.trajectory_id = std::move(arg);
    return Init_PositionCommand_trajectory_flag(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

class Init_PositionCommand_kv
{
public:
  explicit Init_PositionCommand_kv(::quadrotor_msgs::msg::PositionCommand & msg)
  : msg_(msg)
  {}
  Init_PositionCommand_trajectory_id kv(::quadrotor_msgs::msg::PositionCommand::_kv_type arg)
  {
    msg_.kv = std::move(arg);
    return Init_PositionCommand_trajectory_id(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

class Init_PositionCommand_kx
{
public:
  explicit Init_PositionCommand_kx(::quadrotor_msgs::msg::PositionCommand & msg)
  : msg_(msg)
  {}
  Init_PositionCommand_kv kx(::quadrotor_msgs::msg::PositionCommand::_kx_type arg)
  {
    msg_.kx = std::move(arg);
    return Init_PositionCommand_kv(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

class Init_PositionCommand_yaw_dot
{
public:
  explicit Init_PositionCommand_yaw_dot(::quadrotor_msgs::msg::PositionCommand & msg)
  : msg_(msg)
  {}
  Init_PositionCommand_kx yaw_dot(::quadrotor_msgs::msg::PositionCommand::_yaw_dot_type arg)
  {
    msg_.yaw_dot = std::move(arg);
    return Init_PositionCommand_kx(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

class Init_PositionCommand_yaw
{
public:
  explicit Init_PositionCommand_yaw(::quadrotor_msgs::msg::PositionCommand & msg)
  : msg_(msg)
  {}
  Init_PositionCommand_yaw_dot yaw(::quadrotor_msgs::msg::PositionCommand::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PositionCommand_yaw_dot(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

class Init_PositionCommand_acceleration
{
public:
  explicit Init_PositionCommand_acceleration(::quadrotor_msgs::msg::PositionCommand & msg)
  : msg_(msg)
  {}
  Init_PositionCommand_yaw acceleration(::quadrotor_msgs::msg::PositionCommand::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_PositionCommand_yaw(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

class Init_PositionCommand_velocity
{
public:
  explicit Init_PositionCommand_velocity(::quadrotor_msgs::msg::PositionCommand & msg)
  : msg_(msg)
  {}
  Init_PositionCommand_acceleration velocity(::quadrotor_msgs::msg::PositionCommand::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_PositionCommand_acceleration(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

class Init_PositionCommand_position
{
public:
  explicit Init_PositionCommand_position(::quadrotor_msgs::msg::PositionCommand & msg)
  : msg_(msg)
  {}
  Init_PositionCommand_velocity position(::quadrotor_msgs::msg::PositionCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_PositionCommand_velocity(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

class Init_PositionCommand_header
{
public:
  Init_PositionCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionCommand_position header(::quadrotor_msgs::msg::PositionCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PositionCommand_position(msg_);
  }

private:
  ::quadrotor_msgs::msg::PositionCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quadrotor_msgs::msg::PositionCommand>()
{
  return quadrotor_msgs::msg::builder::Init_PositionCommand_header();
}

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POSITION_COMMAND__BUILDER_HPP_
