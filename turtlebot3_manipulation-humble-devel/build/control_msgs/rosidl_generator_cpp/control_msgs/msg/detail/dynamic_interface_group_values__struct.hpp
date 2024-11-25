// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from control_msgs:msg/DynamicInterfaceGroupValues.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__STRUCT_HPP_
#define CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__STRUCT_HPP_

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
// Member 'interface_values'
#include "control_msgs/msg/detail/interface_value__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__control_msgs__msg__DynamicInterfaceGroupValues __attribute__((deprecated))
#else
# define DEPRECATED__control_msgs__msg__DynamicInterfaceGroupValues __declspec(deprecated)
#endif

namespace control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DynamicInterfaceGroupValues_
{
  using Type = DynamicInterfaceGroupValues_<ContainerAllocator>;

  explicit DynamicInterfaceGroupValues_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit DynamicInterfaceGroupValues_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _interface_groups_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _interface_groups_type interface_groups;
  using _interface_values_type =
    std::vector<control_msgs::msg::InterfaceValue_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<control_msgs::msg::InterfaceValue_<ContainerAllocator>>>;
  _interface_values_type interface_values;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__interface_groups(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->interface_groups = _arg;
    return *this;
  }
  Type & set__interface_values(
    const std::vector<control_msgs::msg::InterfaceValue_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<control_msgs::msg::InterfaceValue_<ContainerAllocator>>> & _arg)
  {
    this->interface_values = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator> *;
  using ConstRawPtr =
    const control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__control_msgs__msg__DynamicInterfaceGroupValues
    std::shared_ptr<control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__control_msgs__msg__DynamicInterfaceGroupValues
    std::shared_ptr<control_msgs::msg::DynamicInterfaceGroupValues_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DynamicInterfaceGroupValues_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->interface_groups != other.interface_groups) {
      return false;
    }
    if (this->interface_values != other.interface_values) {
      return false;
    }
    return true;
  }
  bool operator!=(const DynamicInterfaceGroupValues_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DynamicInterfaceGroupValues_

// alias to use template instance with default allocator
using DynamicInterfaceGroupValues =
  control_msgs::msg::DynamicInterfaceGroupValues_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__STRUCT_HPP_
