// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from control_msgs:msg/DynamicInterfaceGroupValues.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__BUILDER_HPP_
#define CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "control_msgs/msg/detail/dynamic_interface_group_values__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace control_msgs
{

namespace msg
{

namespace builder
{

class Init_DynamicInterfaceGroupValues_interface_values
{
public:
  explicit Init_DynamicInterfaceGroupValues_interface_values(::control_msgs::msg::DynamicInterfaceGroupValues & msg)
  : msg_(msg)
  {}
  ::control_msgs::msg::DynamicInterfaceGroupValues interface_values(::control_msgs::msg::DynamicInterfaceGroupValues::_interface_values_type arg)
  {
    msg_.interface_values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_msgs::msg::DynamicInterfaceGroupValues msg_;
};

class Init_DynamicInterfaceGroupValues_interface_groups
{
public:
  explicit Init_DynamicInterfaceGroupValues_interface_groups(::control_msgs::msg::DynamicInterfaceGroupValues & msg)
  : msg_(msg)
  {}
  Init_DynamicInterfaceGroupValues_interface_values interface_groups(::control_msgs::msg::DynamicInterfaceGroupValues::_interface_groups_type arg)
  {
    msg_.interface_groups = std::move(arg);
    return Init_DynamicInterfaceGroupValues_interface_values(msg_);
  }

private:
  ::control_msgs::msg::DynamicInterfaceGroupValues msg_;
};

class Init_DynamicInterfaceGroupValues_header
{
public:
  Init_DynamicInterfaceGroupValues_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DynamicInterfaceGroupValues_interface_groups header(::control_msgs::msg::DynamicInterfaceGroupValues::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DynamicInterfaceGroupValues_interface_groups(msg_);
  }

private:
  ::control_msgs::msg::DynamicInterfaceGroupValues msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_msgs::msg::DynamicInterfaceGroupValues>()
{
  return control_msgs::msg::builder::Init_DynamicInterfaceGroupValues_header();
}

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__BUILDER_HPP_
