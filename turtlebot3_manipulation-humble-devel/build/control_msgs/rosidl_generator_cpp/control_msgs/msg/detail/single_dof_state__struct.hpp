// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from control_msgs:msg/SingleDOFState.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE__STRUCT_HPP_
#define CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__control_msgs__msg__SingleDOFState __attribute__((deprecated))
#else
# define DEPRECATED__control_msgs__msg__SingleDOFState __declspec(deprecated)
#endif

namespace control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SingleDOFState_
{
  using Type = SingleDOFState_<ContainerAllocator>;

  explicit SingleDOFState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->reference = 0.0;
      this->feedback = 0.0;
      this->feedback_dot = 0.0;
      this->error = 0.0;
      this->error_dot = 0.0;
      this->time_step = 0.0;
      this->output = 0.0;
    }
  }

  explicit SingleDOFState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->reference = 0.0;
      this->feedback = 0.0;
      this->feedback_dot = 0.0;
      this->error = 0.0;
      this->error_dot = 0.0;
      this->time_step = 0.0;
      this->output = 0.0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _reference_type =
    double;
  _reference_type reference;
  using _feedback_type =
    double;
  _feedback_type feedback;
  using _feedback_dot_type =
    double;
  _feedback_dot_type feedback_dot;
  using _error_type =
    double;
  _error_type error;
  using _error_dot_type =
    double;
  _error_dot_type error_dot;
  using _time_step_type =
    double;
  _time_step_type time_step;
  using _output_type =
    double;
  _output_type output;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__reference(
    const double & _arg)
  {
    this->reference = _arg;
    return *this;
  }
  Type & set__feedback(
    const double & _arg)
  {
    this->feedback = _arg;
    return *this;
  }
  Type & set__feedback_dot(
    const double & _arg)
  {
    this->feedback_dot = _arg;
    return *this;
  }
  Type & set__error(
    const double & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__error_dot(
    const double & _arg)
  {
    this->error_dot = _arg;
    return *this;
  }
  Type & set__time_step(
    const double & _arg)
  {
    this->time_step = _arg;
    return *this;
  }
  Type & set__output(
    const double & _arg)
  {
    this->output = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    control_msgs::msg::SingleDOFState_<ContainerAllocator> *;
  using ConstRawPtr =
    const control_msgs::msg::SingleDOFState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<control_msgs::msg::SingleDOFState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<control_msgs::msg::SingleDOFState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::SingleDOFState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::SingleDOFState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      control_msgs::msg::SingleDOFState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<control_msgs::msg::SingleDOFState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<control_msgs::msg::SingleDOFState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<control_msgs::msg::SingleDOFState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__control_msgs__msg__SingleDOFState
    std::shared_ptr<control_msgs::msg::SingleDOFState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__control_msgs__msg__SingleDOFState
    std::shared_ptr<control_msgs::msg::SingleDOFState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SingleDOFState_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->reference != other.reference) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    if (this->feedback_dot != other.feedback_dot) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->error_dot != other.error_dot) {
      return false;
    }
    if (this->time_step != other.time_step) {
      return false;
    }
    if (this->output != other.output) {
      return false;
    }
    return true;
  }
  bool operator!=(const SingleDOFState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SingleDOFState_

// alias to use template instance with default allocator
using SingleDOFState =
  control_msgs::msg::SingleDOFState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE__STRUCT_HPP_
