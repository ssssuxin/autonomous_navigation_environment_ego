// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from traj_utils:msg/Bspline.idl
// generated code does not contain a copyright notice

#ifndef TRAJ_UTILS__MSG__DETAIL__BSPLINE__STRUCT_HPP_
#define TRAJ_UTILS__MSG__DETAIL__BSPLINE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'start_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'pos_pts'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__traj_utils__msg__Bspline __attribute__((deprecated))
#else
# define DEPRECATED__traj_utils__msg__Bspline __declspec(deprecated)
#endif

namespace traj_utils
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Bspline_
{
  using Type = Bspline_<ContainerAllocator>;

  explicit Bspline_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id = 0l;
      this->order = 0l;
      this->traj_id = 0ll;
      this->yaw_dt = 0.0;
    }
  }

  explicit Bspline_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id = 0l;
      this->order = 0l;
      this->traj_id = 0ll;
      this->yaw_dt = 0.0;
    }
  }

  // field types and members
  using _drone_id_type =
    int32_t;
  _drone_id_type drone_id;
  using _order_type =
    int32_t;
  _order_type order;
  using _traj_id_type =
    int64_t;
  _traj_id_type traj_id;
  using _start_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _start_time_type start_time;
  using _knots_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _knots_type knots;
  using _pos_pts_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point_<ContainerAllocator>>::other>;
  _pos_pts_type pos_pts;
  using _yaw_pts_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _yaw_pts_type yaw_pts;
  using _yaw_dt_type =
    double;
  _yaw_dt_type yaw_dt;

  // setters for named parameter idiom
  Type & set__drone_id(
    const int32_t & _arg)
  {
    this->drone_id = _arg;
    return *this;
  }
  Type & set__order(
    const int32_t & _arg)
  {
    this->order = _arg;
    return *this;
  }
  Type & set__traj_id(
    const int64_t & _arg)
  {
    this->traj_id = _arg;
    return *this;
  }
  Type & set__start_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->start_time = _arg;
    return *this;
  }
  Type & set__knots(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->knots = _arg;
    return *this;
  }
  Type & set__pos_pts(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename ContainerAllocator::template rebind<geometry_msgs::msg::Point_<ContainerAllocator>>::other> & _arg)
  {
    this->pos_pts = _arg;
    return *this;
  }
  Type & set__yaw_pts(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->yaw_pts = _arg;
    return *this;
  }
  Type & set__yaw_dt(
    const double & _arg)
  {
    this->yaw_dt = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    traj_utils::msg::Bspline_<ContainerAllocator> *;
  using ConstRawPtr =
    const traj_utils::msg::Bspline_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<traj_utils::msg::Bspline_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<traj_utils::msg::Bspline_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      traj_utils::msg::Bspline_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<traj_utils::msg::Bspline_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      traj_utils::msg::Bspline_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<traj_utils::msg::Bspline_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<traj_utils::msg::Bspline_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<traj_utils::msg::Bspline_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__traj_utils__msg__Bspline
    std::shared_ptr<traj_utils::msg::Bspline_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__traj_utils__msg__Bspline
    std::shared_ptr<traj_utils::msg::Bspline_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bspline_ & other) const
  {
    if (this->drone_id != other.drone_id) {
      return false;
    }
    if (this->order != other.order) {
      return false;
    }
    if (this->traj_id != other.traj_id) {
      return false;
    }
    if (this->start_time != other.start_time) {
      return false;
    }
    if (this->knots != other.knots) {
      return false;
    }
    if (this->pos_pts != other.pos_pts) {
      return false;
    }
    if (this->yaw_pts != other.yaw_pts) {
      return false;
    }
    if (this->yaw_dt != other.yaw_dt) {
      return false;
    }
    return true;
  }
  bool operator!=(const Bspline_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bspline_

// alias to use template instance with default allocator
using Bspline =
  traj_utils::msg::Bspline_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace traj_utils

#endif  // TRAJ_UTILS__MSG__DETAIL__BSPLINE__STRUCT_HPP_
