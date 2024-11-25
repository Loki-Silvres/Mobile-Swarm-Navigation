// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from control_msgs:msg/SingleDOFStateStamped.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE_STAMPED__STRUCT_HPP_
#define CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE_STAMPED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'dof_state'
#include "control_msgs/msg/detail/single_dof_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__control_msgs__msg__SingleDOFStateStamped __attribute__((deprecated))
#else
# define DEPRECATED__control_msgs__msg__SingleDOFStateStamped __declspec(deprecated)
#endif

namespace control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SingleDOFStateStamped_
{
  using Type = SingleDOFStateStamped_<ContainerAllocator>;

  explicit SingleDOFStateStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    dof_state(_init)
  {
    (void)_init;
  }

  explicit SingleDOFStateStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    dof_state(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _dof_state_type =
    control_msgs::msg::SingleDOFState_<ContainerAllocator>;
  _dof_state_type dof_state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__dof_state(
    const control_msgs::msg::SingleDOFState_<ContainerAllocator> & _arg)
  {
    this->dof_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__control_msgs__msg__SingleDOFStateStamped
    std::shared_ptr<control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__control_msgs__msg__SingleDOFStateStamped
    std::shared_ptr<control_msgs::msg::SingleDOFStateStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SingleDOFStateStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->dof_state != other.dof_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SingleDOFStateStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SingleDOFStateStamped_

// alias to use template instance with default allocator
using SingleDOFStateStamped =
  control_msgs::msg::SingleDOFStateStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE_STAMPED__STRUCT_HPP_
