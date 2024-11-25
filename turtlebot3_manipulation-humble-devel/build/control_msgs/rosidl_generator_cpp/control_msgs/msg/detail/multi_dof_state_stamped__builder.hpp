// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from control_msgs:msg/MultiDOFStateStamped.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_STATE_STAMPED__BUILDER_HPP_
#define CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_STATE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "control_msgs/msg/detail/multi_dof_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace control_msgs
{

namespace msg
{

namespace builder
{

class Init_MultiDOFStateStamped_dof_states
{
public:
  explicit Init_MultiDOFStateStamped_dof_states(::control_msgs::msg::MultiDOFStateStamped & msg)
  : msg_(msg)
  {}
  ::control_msgs::msg::MultiDOFStateStamped dof_states(::control_msgs::msg::MultiDOFStateStamped::_dof_states_type arg)
  {
    msg_.dof_states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_msgs::msg::MultiDOFStateStamped msg_;
};

class Init_MultiDOFStateStamped_header
{
public:
  Init_MultiDOFStateStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiDOFStateStamped_dof_states header(::control_msgs::msg::MultiDOFStateStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MultiDOFStateStamped_dof_states(msg_);
  }

private:
  ::control_msgs::msg::MultiDOFStateStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_msgs::msg::MultiDOFStateStamped>()
{
  return control_msgs::msg::builder::Init_MultiDOFStateStamped_header();
}

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__MULTI_DOF_STATE_STAMPED__BUILDER_HPP_
