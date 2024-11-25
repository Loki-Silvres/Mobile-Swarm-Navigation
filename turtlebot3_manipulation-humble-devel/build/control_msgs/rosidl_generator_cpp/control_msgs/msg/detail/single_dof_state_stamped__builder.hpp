// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from control_msgs:msg/SingleDOFStateStamped.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE_STAMPED__BUILDER_HPP_
#define CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "control_msgs/msg/detail/single_dof_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace control_msgs
{

namespace msg
{

namespace builder
{

class Init_SingleDOFStateStamped_dof_state
{
public:
  explicit Init_SingleDOFStateStamped_dof_state(::control_msgs::msg::SingleDOFStateStamped & msg)
  : msg_(msg)
  {}
  ::control_msgs::msg::SingleDOFStateStamped dof_state(::control_msgs::msg::SingleDOFStateStamped::_dof_state_type arg)
  {
    msg_.dof_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFStateStamped msg_;
};

class Init_SingleDOFStateStamped_header
{
public:
  Init_SingleDOFStateStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SingleDOFStateStamped_dof_state header(::control_msgs::msg::SingleDOFStateStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SingleDOFStateStamped_dof_state(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFStateStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_msgs::msg::SingleDOFStateStamped>()
{
  return control_msgs::msg::builder::Init_SingleDOFStateStamped_header();
}

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE_STAMPED__BUILDER_HPP_
