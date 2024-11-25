// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from control_msgs:msg/MultiDOFCommand.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_COMMAND__STRUCT_HPP_
#define CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__control_msgs__msg__MultiDOFCommand __attribute__((deprecated))
#else
# define DEPRECATED__control_msgs__msg__MultiDOFCommand __declspec(deprecated)
#endif

namespace control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiDOFCommand_
{
  using Type = MultiDOFCommand_<ContainerAllocator>;

  explicit MultiDOFCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MultiDOFCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _dof_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _dof_names_type dof_names;
  using _values_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _values_type values;
  using _values_dot_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _values_dot_type values_dot;

  // setters for named parameter idiom
  Type & set__dof_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->dof_names = _arg;
    return *this;
  }
  Type & set__values(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->values = _arg;
    return *this;
  }
  Type & set__values_dot(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->values_dot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    control_msgs::msg::MultiDOFCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const control_msgs::msg::MultiDOFCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<control_msgs::msg::MultiDOFCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<control_msgs::msg::MultiDOFCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::MultiDOFCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::MultiDOFCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::MultiDOFCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::MultiDOFCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<control_msgs::msg::MultiDOFCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<control_msgs::msg::MultiDOFCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__control_msgs__msg__MultiDOFCommand
    std::shared_ptr<control_msgs::msg::MultiDOFCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__control_msgs__msg__MultiDOFCommand
    std::shared_ptr<control_msgs::msg::MultiDOFCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiDOFCommand_ & other) const
  {
    if (this->dof_names != other.dof_names) {
      return false;
    }
    if (this->values != other.values) {
      return false;
    }
    if (this->values_dot != other.values_dot) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiDOFCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiDOFCommand_

// alias to use template instance with default allocator
using MultiDOFCommand =
  control_msgs::msg::MultiDOFCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_COMMAND__STRUCT_HPP_
