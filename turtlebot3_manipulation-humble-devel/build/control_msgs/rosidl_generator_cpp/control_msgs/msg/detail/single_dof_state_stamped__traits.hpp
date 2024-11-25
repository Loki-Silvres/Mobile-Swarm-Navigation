// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from control_msgs:msg/SingleDOFStateStamped.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE_STAMPED__TRAITS_HPP_
#define CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "control_msgs/msg/detail/single_dof_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'dof_state'
#include "control_msgs/msg/detail/single_dof_state__traits.hpp"

namespace control_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SingleDOFStateStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: dof_state
  {
    out << "dof_state: ";
    to_flow_style_yaml(msg.dof_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SingleDOFStateStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: dof_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dof_state:\n";
    to_block_style_yaml(msg.dof_state, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SingleDOFStateStamped & msg, bool use_flow_style = false)
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
  const control_msgs::msg::SingleDOFStateStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const control_msgs::msg::SingleDOFStateStamped & msg)
{
  return control_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<control_msgs::msg::SingleDOFStateStamped>()
{
  return "control_msgs::msg::SingleDOFStateStamped";
}

template<>
inline const char * name<control_msgs::msg::SingleDOFStateStamped>()
{
  return "control_msgs/msg/SingleDOFStateStamped";
}

template<>
struct has_fixed_size<control_msgs::msg::SingleDOFStateStamped>
  : std::integral_constant<bool, has_fixed_size<control_msgs::msg::SingleDOFState>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<control_msgs::msg::SingleDOFStateStamped>
  : std::integral_constant<bool, has_bounded_size<control_msgs::msg::SingleDOFState>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<control_msgs::msg::SingleDOFStateStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROL_MSGS__MSG__DETAIL__SINGLE_DOF_STATE_STAMPED__TRAITS_HPP_
