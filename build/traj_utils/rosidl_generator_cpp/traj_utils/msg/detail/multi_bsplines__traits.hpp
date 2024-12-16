// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__TRAITS_HPP_
#define TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__TRAITS_HPP_

#include "traj_utils/msg/detail/multi_bsplines__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<traj_utils::msg::MultiBsplines>()
{
  return "traj_utils::msg::MultiBsplines";
}

template<>
inline const char * name<traj_utils::msg::MultiBsplines>()
{
  return "traj_utils/msg/MultiBsplines";
}

template<>
struct has_fixed_size<traj_utils::msg::MultiBsplines>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<traj_utils::msg::MultiBsplines>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<traj_utils::msg::MultiBsplines>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__TRAITS_HPP_
