// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from control_msgs:msg/MultiDOFCommand.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_COMMAND__BUILDER_HPP_
#define CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "control_msgs/msg/detail/multi_dof_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace control_msgs
{

namespace msg
{

namespace builder
{

class Init_MultiDOFCommand_values_dot
{
public:
  explicit Init_MultiDOFCommand_values_dot(::control_msgs::msg::MultiDOFCommand & msg)
  : msg_(msg)
  {}
  ::control_msgs::msg::MultiDOFCommand values_dot(::control_msgs::msg::MultiDOFCommand::_values_dot_type arg)
  {
    msg_.values_dot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_msgs::msg::MultiDOFCommand msg_;
};

class Init_MultiDOFCommand_values
{
public:
  explicit Init_MultiDOFCommand_values(::control_msgs::msg::MultiDOFCommand & msg)
  : msg_(msg)
  {}
  Init_MultiDOFCommand_values_dot values(::control_msgs::msg::MultiDOFCommand::_values_type arg)
  {
    msg_.values = std::move(arg);
    return Init_MultiDOFCommand_values_dot(msg_);
  }

private:
  ::control_msgs::msg::MultiDOFCommand msg_;
};

class Init_MultiDOFCommand_dof_names
{
public:
  Init_MultiDOFCommand_dof_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiDOFCommand_values dof_names(::control_msgs::msg::MultiDOFCommand::_dof_names_type arg)
  {
    msg_.dof_names = std::move(arg);
    return Init_MultiDOFCommand_values(msg_);
  }

private:
  ::control_msgs::msg::MultiDOFCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_msgs::msg::MultiDOFCommand>()
{
  return control_msgs::msg::builder::Init_MultiDOFCommand_dof_names();
}

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_COMMAND__BUILDER_HPP_
