// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/PositionCommand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__POSITION_COMMAND__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__POSITION_COMMAND__STRUCT_HPP_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quadrotor_msgs__msg__PositionCommand __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__PositionCommand __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PositionCommand_
{
  using Type = PositionCommand_<ContainerAllocator>;

  explicit PositionCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init),
    acceleration(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->yaw_dot = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->kx.begin(), this->kx.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->kv.begin(), this->kv.end(), 0.0);
      this->trajectory_id = 0ul;
      this->trajectory_flag = 0;
    }
  }

  explicit PositionCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init),
    acceleration(_alloc, _init),
    kx(_alloc),
    kv(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
      this->yaw_dot = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->kx.begin(), this->kx.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->kv.begin(), this->kv.end(), 0.0);
      this->trajectory_id = 0ul;
      this->trajectory_flag = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _yaw_dot_type =
    double;
  _yaw_dot_type yaw_dot;
  using _kx_type =
    std::array<double, 3>;
  _kx_type kx;
  using _kv_type =
    std::array<double, 3>;
  _kv_type kv;
  using _trajectory_id_type =
    uint32_t;
  _trajectory_id_type trajectory_id;
  using _trajectory_flag_type =
    uint8_t;
  _trajectory_flag_type trajectory_flag;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__yaw_dot(
    const double & _arg)
  {
    this->yaw_dot = _arg;
    return *this;
  }
  Type & set__kx(
    const std::array<double, 3> & _arg)
  {
    this->kx = _arg;
    return *this;
  }
  Type & set__kv(
    const std::array<double, 3> & _arg)
  {
    this->kv = _arg;
    return *this;
  }
  Type & set__trajectory_id(
    const uint32_t & _arg)
  {
    this->trajectory_id = _arg;
    return *this;
  }
  Type & set__trajectory_flag(
    const uint8_t & _arg)
  {
    this->trajectory_flag = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t TRAJECTORY_STATUS_EMPTY =
    0u;
  static constexpr uint8_t TRAJECTORY_STATUS_READY =
    1u;
  static constexpr uint8_t TRAJECTORY_STATUS_COMPLETED =
    3u;
  static constexpr uint8_t TRAJECTROY_STATUS_ABORT =
    4u;
  static constexpr uint8_t TRAJECTORY_STATUS_ILLEGAL_START =
    5u;
  static constexpr uint8_t TRAJECTORY_STATUS_ILLEGAL_FINAL =
    6u;
  static constexpr uint8_t TRAJECTORY_STATUS_IMPOSSIBLE =
    7u;

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::PositionCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::PositionCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::PositionCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::PositionCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::PositionCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::PositionCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::PositionCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::PositionCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::PositionCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::PositionCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__PositionCommand
    std::shared_ptr<quadrotor_msgs::msg::PositionCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__PositionCommand
    std::shared_ptr<quadrotor_msgs::msg::PositionCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PositionCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->yaw_dot != other.yaw_dot) {
      return false;
    }
    if (this->kx != other.kx) {
      return false;
    }
    if (this->kv != other.kv) {
      return false;
    }
    if (this->trajectory_id != other.trajectory_id) {
      return false;
    }
    if (this->trajectory_flag != other.trajectory_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const PositionCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PositionCommand_

// alias to use template instance with default allocator
using PositionCommand =
  quadrotor_msgs::msg::PositionCommand_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t PositionCommand_<ContainerAllocator>::TRAJECTORY_STATUS_EMPTY;
template<typename ContainerAllocator>
constexpr uint8_t PositionCommand_<ContainerAllocator>::TRAJECTORY_STATUS_READY;
template<typename ContainerAllocator>
constexpr uint8_t PositionCommand_<ContainerAllocator>::TRAJECTORY_STATUS_COMPLETED;
template<typename ContainerAllocator>
constexpr uint8_t PositionCommand_<ContainerAllocator>::TRAJECTROY_STATUS_ABORT;
template<typename ContainerAllocator>
constexpr uint8_t PositionCommand_<ContainerAllocator>::TRAJECTORY_STATUS_ILLEGAL_START;
template<typename ContainerAllocator>
constexpr uint8_t PositionCommand_<ContainerAllocator>::TRAJECTORY_STATUS_ILLEGAL_FINAL;
template<typename ContainerAllocator>
constexpr uint8_t PositionCommand_<ContainerAllocator>::TRAJECTORY_STATUS_IMPOSSIBLE;

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POSITION_COMMAND__STRUCT_HPP_
