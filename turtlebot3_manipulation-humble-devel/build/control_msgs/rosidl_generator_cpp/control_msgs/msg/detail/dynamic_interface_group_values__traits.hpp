// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from control_msgs:msg/DynamicInterfaceGroupValues.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__TRAITS_HPP_
#define CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "control_msgs/msg/detail/dynamic_interface_group_values__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'interface_values'
#include "control_msgs/msg/detail/interface_value__traits.hpp"

namespace control_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DynamicInterfaceGroupValues & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: interface_groups
  {
    if (msg.interface_groups.size() == 0) {
      out << "interface_groups: []";
    } else {
      out << "interface_groups: [";
      size_t pending_items = msg.interface_groups.size();
      for (auto item : msg.interface_groups) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: interface_values
  {
    if (msg.interface_values.size() == 0) {
      out << "interface_values: []";
    } else {
      out << "interface_values: [";
      size_t pending_items = msg.interface_values.size();
      for (auto item : msg.interface_values) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DynamicInterfaceGroupValues & msg,
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

  // member: interface_groups
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.interface_groups.size() == 0) {
      out << "interface_groups: []\n";
    } else {
      out << "interface_groups:\n";
      for (auto item : msg.interface_groups) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: interface_values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.interface_values.size() == 0) {
      out << "interface_values: []\n";
    } else {
      out << "interface_values:\n";
      for (auto item : msg.interface_values) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DynamicInterfaceGroupValues & msg, bool use_flow_style = false)
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
  const control_msgs::msg::DynamicInterfaceGroupValues & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const control_msgs::msg::DynamicInterfaceGroupValues & msg)
{
  return control_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<control_msgs::msg::DynamicInterfaceGroupValues>()
{
  return "control_msgs::msg::DynamicInterfaceGroupValues";
}

template<>
inline const char * name<control_msgs::msg::DynamicInterfaceGroupValues>()
{
  return "control_msgs/msg/DynamicInterfaceGroupValues";
}

template<>
struct has_fixed_size<control_msgs::msg::DynamicInterfaceGroupValues>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<control_msgs::msg::DynamicInterfaceGroupValues>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<control_msgs::msg::DynamicInterfaceGroupValues>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__TRAITS_HPP_
