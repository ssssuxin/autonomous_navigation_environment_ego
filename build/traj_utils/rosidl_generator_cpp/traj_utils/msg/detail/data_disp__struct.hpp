// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from traj_utils:msg/DataDisp.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__DATA_DISP__STRUCT_HPP_
#define TRAJ_UTILS__MSG__DETAIL__DATA_DISP__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__traj_utils__msg__DataDisp __attribute__((deprecated))
#else
# define DEPRECATED__traj_utils__msg__DataDisp __declspec(deprecated)
#endif

namespace traj_utils
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DataDisp_
{
  using Type = DataDisp_<ContainerAllocator>;

  explicit DataDisp_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0.0;
      this->b = 0.0;
      this->c = 0.0;
      this->d = 0.0;
      this->e = 0.0;
    }
  }

  explicit DataDisp_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0.0;
      this->b = 0.0;
      this->c = 0.0;
      this->d = 0.0;
      this->e = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _a_type =
    double;
  _a_type a;
  using _b_type =
    double;
  _b_type b;
  using _c_type =
    double;
  _c_type c;
  using _d_type =
    double;
  _d_type d;
  using _e_type =
    double;
  _e_type e;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__a(
    const double & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const double & _arg)
  {
    this->b = _arg;
    return *this;
  }
  Type & set__c(
    const double & _arg)
  {
    this->c = _arg;
    return *this;
  }
  Type & set__d(
    const double & _arg)
  {
    this->d = _arg;
    return *this;
  }
  Type & set__e(
    const double & _arg)
  {
    this->e = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    traj_utils::msg::DataDisp_<ContainerAllocator> *;
  using ConstRawPtr =
    const traj_utils::msg::DataDisp_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<traj_utils::msg::DataDisp_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<traj_utils::msg::DataDisp_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      traj_utils::msg::DataDisp_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<traj_utils::msg::DataDisp_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      traj_utils::msg::DataDisp_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<traj_utils::msg::DataDisp_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<traj_utils::msg::DataDisp_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<traj_utils::msg::DataDisp_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__traj_utils__msg__DataDisp
    std::shared_ptr<traj_utils::msg::DataDisp_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__traj_utils__msg__DataDisp
    std::shared_ptr<traj_utils::msg::DataDisp_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DataDisp_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    if (this->c != other.c) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    if (this->e != other.e) {
      return false;
    }
    return true;
  }
  bool operator!=(const DataDisp_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DataDisp_

// alias to use template instance with default allocator
using DataDisp =
  traj_utils::msg::DataDisp_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace traj_utils

#endif  // TRAJ_UTILS__MSG__DETAIL__DATA_DISP__STRUCT_HPP_
