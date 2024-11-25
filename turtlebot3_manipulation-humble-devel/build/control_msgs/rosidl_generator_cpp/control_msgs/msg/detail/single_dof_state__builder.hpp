// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from control_msgs:msg/SingleDOFState.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE__BUILDER_HPP_
#define CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "control_msgs/msg/detail/single_dof_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace control_msgs
{

namespace msg
{

namespace builder
{

class Init_SingleDOFState_output
{
public:
  explicit Init_SingleDOFState_output(::control_msgs::msg::SingleDOFState & msg)
  : msg_(msg)
  {}
  ::control_msgs::msg::SingleDOFState output(::control_msgs::msg::SingleDOFState::_output_type arg)
  {
    msg_.output = std::move(arg);
    return std::move(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFState msg_;
};

class Init_SingleDOFState_time_step
{
public:
  explicit Init_SingleDOFState_time_step(::control_msgs::msg::SingleDOFState & msg)
  : msg_(msg)
  {}
  Init_SingleDOFState_output time_step(::control_msgs::msg::SingleDOFState::_time_step_type arg)
  {
    msg_.time_step = std::move(arg);
    return Init_SingleDOFState_output(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFState msg_;
};

class Init_SingleDOFState_error_dot
{
public:
  explicit Init_SingleDOFState_error_dot(::control_msgs::msg::SingleDOFState & msg)
  : msg_(msg)
  {}
  Init_SingleDOFState_time_step error_dot(::control_msgs::msg::SingleDOFState::_error_dot_type arg)
  {
    msg_.error_dot = std::move(arg);
    return Init_SingleDOFState_time_step(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFState msg_;
};

class Init_SingleDOFState_error
{
public:
  explicit Init_SingleDOFState_error(::control_msgs::msg::SingleDOFState & msg)
  : msg_(msg)
  {}
  Init_SingleDOFState_error_dot error(::control_msgs::msg::SingleDOFState::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_SingleDOFState_error_dot(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFState msg_;
};

class Init_SingleDOFState_feedback_dot
{
public:
  explicit Init_SingleDOFState_feedback_dot(::control_msgs::msg::SingleDOFState & msg)
  : msg_(msg)
  {}
  Init_SingleDOFState_error feedback_dot(::control_msgs::msg::SingleDOFState::_feedback_dot_type arg)
  {
    msg_.feedback_dot = std::move(arg);
    return Init_SingleDOFState_error(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFState msg_;
};

class Init_SingleDOFState_feedback
{
public:
  explicit Init_SingleDOFState_feedback(::control_msgs::msg::SingleDOFState & msg)
  : msg_(msg)
  {}
  Init_SingleDOFState_feedback_dot feedback(::control_msgs::msg::SingleDOFState::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return Init_SingleDOFState_feedback_dot(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFState msg_;
};

class Init_SingleDOFState_reference
{
public:
  explicit Init_SingleDOFState_reference(::control_msgs::msg::SingleDOFState & msg)
  : msg_(msg)
  {}
  Init_SingleDOFState_feedback reference(::control_msgs::msg::SingleDOFState::_reference_type arg)
  {
    msg_.reference = std::move(arg);
    return Init_SingleDOFState_feedback(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFState msg_;
};

class Init_SingleDOFState_name
{
public:
  Init_SingleDOFState_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SingleDOFState_reference name(::control_msgs::msg::SingleDOFState::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_SingleDOFState_reference(msg_);
  }

private:
  ::control_msgs::msg::SingleDOFState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::control_msgs::msg::SingleDOFState>()
{
  return control_msgs::msg::builder::Init_SingleDOFState_name();
}

}  // namespace control_msgs

#endif  // CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE__BUILDER_HPP_
