// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from control_msgs:msg/MultiDOFCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "control_msgs/msg/detail/multi_dof_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace control_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MultiDOFCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) control_msgs::msg::MultiDOFCommand(_init);
}

void MultiDOFCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<control_msgs::msg::MultiDOFCommand *>(message_memory);
  typed_message->~MultiDOFCommand();
}

size_t size_function__MultiDOFCommand__dof_names(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiDOFCommand__dof_names(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiDOFCommand__dof_names(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__MultiDOFCommand__dof_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__MultiDOFCommand__dof_names(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__MultiDOFCommand__dof_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__MultiDOFCommand__dof_names(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__MultiDOFCommand__dof_names(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MultiDOFCommand__values(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiDOFCommand__values(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiDOFCommand__values(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__MultiDOFCommand__values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MultiDOFCommand__values(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MultiDOFCommand__values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MultiDOFCommand__values(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__MultiDOFCommand__values(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MultiDOFCommand__values_dot(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiDOFCommand__values_dot(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiDOFCommand__values_dot(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__MultiDOFCommand__values_dot(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MultiDOFCommand__values_dot(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MultiDOFCommand__values_dot(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MultiDOFCommand__values_dot(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__MultiDOFCommand__values_dot(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MultiDOFCommand_message_member_array[3] = {
  {
    "dof_names",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs::msg::MultiDOFCommand, dof_names),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiDOFCommand__dof_names,  // size() function pointer
    get_const_function__MultiDOFCommand__dof_names,  // get_const(index) function pointer
    get_function__MultiDOFCommand__dof_names,  // get(index) function pointer
    fetch_function__MultiDOFCommand__dof_names,  // fetch(index, &value) function pointer
    assign_function__MultiDOFCommand__dof_names,  // assign(index, value) function pointer
    resize_function__MultiDOFCommand__dof_names  // resize(index) function pointer
  },
  {
    "values",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs::msg::MultiDOFCommand, values),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiDOFCommand__values,  // size() function pointer
    get_const_function__MultiDOFCommand__values,  // get_const(index) function pointer
    get_function__MultiDOFCommand__values,  // get(index) function pointer
    fetch_function__MultiDOFCommand__values,  // fetch(index, &value) function pointer
    assign_function__MultiDOFCommand__values,  // assign(index, value) function pointer
    resize_function__MultiDOFCommand__values  // resize(index) function pointer
  },
  {
    "values_dot",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs::msg::MultiDOFCommand, values_dot),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiDOFCommand__values_dot,  // size() function pointer
    get_const_function__MultiDOFCommand__values_dot,  // get_const(index) function pointer
    get_function__MultiDOFCommand__values_dot,  // get(index) function pointer
    fetch_function__MultiDOFCommand__values_dot,  // fetch(index, &value) function pointer
    assign_function__MultiDOFCommand__values_dot,  // assign(index, value) function pointer
    resize_function__MultiDOFCommand__values_dot  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MultiDOFCommand_message_members = {
  "control_msgs::msg",  // message namespace
  "MultiDOFCommand",  // message name
  3,  // number of fields
  sizeof(control_msgs::msg::MultiDOFCommand),
  MultiDOFCommand_message_member_array,  // message members
  MultiDOFCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  MultiDOFCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MultiDOFCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MultiDOFCommand_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace control_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<control_msgs::msg::MultiDOFCommand>()
{
  return &::control_msgs::msg::rosidl_typesupport_introspection_cpp::MultiDOFCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, msg, MultiDOFCommand)() {
  return &::control_msgs::msg::rosidl_typesupport_introspection_cpp::MultiDOFCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
