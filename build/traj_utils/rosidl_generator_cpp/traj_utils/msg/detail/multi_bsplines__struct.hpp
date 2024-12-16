// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from traj_utils:msg/MultiBsplines.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__STRUCT_HPP_
#define TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'traj'
#include "traj_utils/msg/detail/bspline__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__traj_utils__msg__MultiBsplines __attribute__((deprecated))
#else
# define DEPRECATED__traj_utils__msg__MultiBsplines __declspec(deprecated)
#endif

namespace traj_utils
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiBsplines_
{
  using Type = MultiBsplines_<ContainerAllocator>;

  explicit MultiBsplines_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id_from = 0l;
    }
  }

  explicit MultiBsplines_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id_from = 0l;
    }
  }

  // field types and members
  using _drone_id_from_type =
    int32_t;
  _drone_id_from_type drone_id_from;
  using _traj_type =
    std::vector<traj_utils::msg::Bspline_<ContainerAllocator>, typename ContainerAllocator::template rebind<traj_utils::msg::Bspline_<ContainerAllocator>>::other>;
  _traj_type traj;

  // setters for named parameter idiom
  Type & set__drone_id_from(
    const int32_t & _arg)
  {
    this->drone_id_from = _arg;
    return *this;
  }
  Type & set__traj(
    const std::vector<traj_utils::msg::Bspline_<ContainerAllocator>, typename ContainerAllocator::template rebind<traj_utils::msg::Bspline_<ContainerAllocator>>::other> & _arg)
  {
    this->traj = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    traj_utils::msg::MultiBsplines_<ContainerAllocator> *;
  using ConstRawPtr =
    const traj_utils::msg::MultiBsplines_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<traj_utils::msg::MultiBsplines_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<traj_utils::msg::MultiBsplines_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      traj_utils::msg::MultiBsplines_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<traj_utils::msg::MultiBsplines_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      traj_utils::msg::MultiBsplines_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<traj_utils::msg::MultiBsplines_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<traj_utils::msg::MultiBsplines_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<traj_utils::msg::MultiBsplines_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__traj_utils__msg__MultiBsplines
    std::shared_ptr<traj_utils::msg::MultiBsplines_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__traj_utils__msg__MultiBsplines
    std::shared_ptr<traj_utils::msg::MultiBsplines_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiBsplines_ & other) const
  {
    if (this->drone_id_from != other.drone_id_from) {
      return false;
    }
    if (this->traj != other.traj) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiBsplines_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiBsplines_

// alias to use template instance with default allocator
using MultiBsplines =
  traj_utils::msg::MultiBsplines_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace traj_utils

#endif  // TRAJ_UTILS__MSG__DETAIL__MULTI_BSPLINES__STRUCT_HPP_
