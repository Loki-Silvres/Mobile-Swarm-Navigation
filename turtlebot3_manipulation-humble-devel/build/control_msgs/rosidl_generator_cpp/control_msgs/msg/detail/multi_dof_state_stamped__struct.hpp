// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from control_msgs:msg/MultiDOFStateStamped.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_STATE_STAMPED__STRUCT_HPP_
#define CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_STATE_STAMPED__STRUCT_HPP_

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
// Member 'dof_states'
#include "control_msgs/msg/detail/single_dof_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__control_msgs__msg__MultiDOFStateStamped __attribute__((deprecated))
#else
# define DEPRECATED__control_msgs__msg__MultiDOFStateStamped __declspec(deprecated)
#endif

namespace control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiDOFStateStamped_
{
  using Type = MultiDOFStateStamped_<ContainerAllocator>;

  explicit MultiDOFStateStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MultiDOFStateStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _dof_states_type =
    std::vector<control_msgs::msg::SingleDOFState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<control_msgs::msg::SingleDOFState_<ContainerAllocator>>>;
  _dof_states_type dof_states;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__dof_states(
    const std::vector<control_msgs::msg::SingleDOFState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<control_msgs::msg::SingleDOFState_<ContainerAllocator>>> & _arg)
  {
    this->dof_states = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__control_msgs__msg__MultiDOFStateStamped
    std::shared_ptr<control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__control_msgs__msg__MultiDOFStateStamped
    std::shared_ptr<control_msgs::msg::MultiDOFStateStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiDOFStateStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->dof_states != other.dof_states) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiDOFStateStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiDOFStateStamped_

// alias to use template instance with default allocator
using MultiDOFStateStamped =
  control_msgs::msg::MultiDOFStateStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_STATE_STAMPED__STRUCT_HPP_
