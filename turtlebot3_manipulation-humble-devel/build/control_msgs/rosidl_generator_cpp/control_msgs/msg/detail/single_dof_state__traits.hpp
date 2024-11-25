// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from control_msgs:msg/SingleDOFState.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE__TRAITS_HPP_
#define CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "control_msgs/msg/detail/single_dof_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace control_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SingleDOFState & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: reference
  {
    out << "reference: ";
    rosidl_generator_traits::value_to_yaml(msg.reference, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback, out);
    out << ", ";
  }

  // member: feedback_dot
  {
    out << "feedback_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback_dot, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << ", ";
  }

  // member: error_dot
  {
    out << "error_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.error_dot, out);
    out << ", ";
  }

  // member: time_step
  {
    out << "time_step: ";
    rosidl_generator_traits::value_to_yaml(msg.time_step, out);
    out << ", ";
  }

  // member: output
  {
    out << "output: ";
    rosidl_generator_traits::value_to_yaml(msg.output, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SingleDOFState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: reference
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reference: ";
    rosidl_generator_traits::value_to_yaml(msg.reference, out);
    out << "\n";
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback, out);
    out << "\n";
  }

  // member: feedback_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback_dot, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << "\n";
  }

  // member: error_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.error_dot, out);
    out << "\n";
  }

  // member: time_step
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_step: ";
    rosidl_generator_traits::value_to_yaml(msg.time_step, out);
    out << "\n";
  }

  // member: output
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output: ";
    rosidl_generator_traits::value_to_yaml(msg.output, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SingleDOFState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace control_msgs

namespace rosidl_generator_traits
{

[[deprecated("use control_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_msgs::msg::SingleDOFState & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const control_msgs::msg::SingleDOFState & msg)
{
  return control_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<control_msgs::msg::SingleDOFState>()
{
  return "control_msgs::msg::SingleDOFState";
}

template<>
inline const char * name<control_msgs::msg::SingleDOFState>()
{
  return "control_msgs/msg/SingleDOFState";
}

template<>
struct has_fixed_size<control_msgs::msg::SingleDOFState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<control_msgs::msg::SingleDOFState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<control_msgs::msg::SingleDOFState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE__TRAITS_HPP_
