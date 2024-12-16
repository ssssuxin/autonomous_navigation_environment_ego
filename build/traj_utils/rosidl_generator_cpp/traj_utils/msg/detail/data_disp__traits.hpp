// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from traj_utils:msg/DataDisp.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__DATA_DISP__TRAITS_HPP_
#define TRAJ_UTILS__MSG__DETAIL__DATA_DISP__TRAITS_HPP_

#include "traj_utils/msg/detail/data_disp__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<traj_utils::msg::DataDisp>()
{
  return "traj_utils::msg::DataDisp";
}

template<>
inline const char * name<traj_utils::msg::DataDisp>()
{
  return "traj_utils/msg/DataDisp";
}

template<>
struct has_fixed_size<traj_utils::msg::DataDisp>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<traj_utils::msg::DataDisp>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<traj_utils::msg::DataDisp>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRAJ_UTILS__MSG__DETAIL__DATA_DISP__TRAITS_HPP_
