// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from control_msgs:msg/DynamicInterfaceGroupValues.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "control_msgs/msg/detail/dynamic_interface_group_values__rosidl_typesupport_introspection_c.h"
#include "control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "control_msgs/msg/detail/dynamic_interface_group_values__functions.h"
#include "control_msgs/msg/detail/dynamic_interface_group_values__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `interface_groups`
#include "rosidl_runtime_c/string_functions.h"
// Member `interface_values`
#include "control_msgs/msg/interface_value.h"
// Member `interface_values`
#include "control_msgs/msg/detail/interface_value__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  control_msgs__msg__DynamicInterfaceGroupValues__init(message_memory);
}

void control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_fini_function(void * message_memory)
{
  control_msgs__msg__DynamicInterfaceGroupValues__fini(message_memory);
}

size_t control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__size_function__DynamicInterfaceGroupValues__interface_groups(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_const_function__DynamicInterfaceGroupValues__interface_groups(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_function__DynamicInterfaceGroupValues__interface_groups(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__fetch_function__DynamicInterfaceGroupValues__interface_groups(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_const_function__DynamicInterfaceGroupValues__interface_groups(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__assign_function__DynamicInterfaceGroupValues__interface_groups(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_function__DynamicInterfaceGroupValues__interface_groups(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__resize_function__DynamicInterfaceGroupValues__interface_groups(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__size_function__DynamicInterfaceGroupValues__interface_values(
  const void * untyped_member)
{
  const control_msgs__msg__InterfaceValue__Sequence * member =
    (const control_msgs__msg__InterfaceValue__Sequence *)(untyped_member);
  return member->size;
}

const void * control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_const_function__DynamicInterfaceGroupValues__interface_values(
  const void * untyped_member, size_t index)
{
  const control_msgs__msg__InterfaceValue__Sequence * member =
    (const control_msgs__msg__InterfaceValue__Sequence *)(untyped_member);
  return &member->data[index];
}

void * control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_function__DynamicInterfaceGroupValues__interface_values(
  void * untyped_member, size_t index)
{
  control_msgs__msg__InterfaceValue__Sequence * member =
    (control_msgs__msg__InterfaceValue__Sequence *)(untyped_member);
  return &member->data[index];
}

void control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__fetch_function__DynamicInterfaceGroupValues__interface_values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const control_msgs__msg__InterfaceValue * item =
    ((const control_msgs__msg__InterfaceValue *)
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_const_function__DynamicInterfaceGroupValues__interface_values(untyped_member, index));
  control_msgs__msg__InterfaceValue * value =
    (control_msgs__msg__InterfaceValue *)(untyped_value);
  *value = *item;
}

void control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__assign_function__DynamicInterfaceGroupValues__interface_values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  control_msgs__msg__InterfaceValue * item =
    ((control_msgs__msg__InterfaceValue *)
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_function__DynamicInterfaceGroupValues__interface_values(untyped_member, index));
  const control_msgs__msg__InterfaceValue * value =
    (const control_msgs__msg__InterfaceValue *)(untyped_value);
  *item = *value;
}

bool control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__resize_function__DynamicInterfaceGroupValues__interface_values(
  void * untyped_member, size_t size)
{
  control_msgs__msg__InterfaceValue__Sequence * member =
    (control_msgs__msg__InterfaceValue__Sequence *)(untyped_member);
  control_msgs__msg__InterfaceValue__Sequence__fini(member);
  return control_msgs__msg__InterfaceValue__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__DynamicInterfaceGroupValues, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "interface_groups",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__DynamicInterfaceGroupValues, interface_groups),  // bytes offset in struct
    NULL,  // default value
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__size_function__DynamicInterfaceGroupValues__interface_groups,  // size() function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_const_function__DynamicInterfaceGroupValues__interface_groups,  // get_const(index) function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_function__DynamicInterfaceGroupValues__interface_groups,  // get(index) function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__fetch_function__DynamicInterfaceGroupValues__interface_groups,  // fetch(index, &value) function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__assign_function__DynamicInterfaceGroupValues__interface_groups,  // assign(index, value) function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__resize_function__DynamicInterfaceGroupValues__interface_groups  // resize(index) function pointer
  },
  {
    "interface_values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__DynamicInterfaceGroupValues, interface_values),  // bytes offset in struct
    NULL,  // default value
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__size_function__DynamicInterfaceGroupValues__interface_values,  // size() function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_const_function__DynamicInterfaceGroupValues__interface_values,  // get_const(index) function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__get_function__DynamicInterfaceGroupValues__interface_values,  // get(index) function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__fetch_function__DynamicInterfaceGroupValues__interface_values,  // fetch(index, &value) function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__assign_function__DynamicInterfaceGroupValues__interface_values,  // assign(index, value) function pointer
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__resize_function__DynamicInterfaceGroupValues__interface_values  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_members = {
  "control_msgs__msg",  // message namespace
  "DynamicInterfaceGroupValues",  // message name
  3,  // number of fields
  sizeof(control_msgs__msg__DynamicInterfaceGroupValues),
  control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_member_array,  // message members
  control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_init_function,  // function to initialize message memory (memory has to be allocated)
  control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_type_support_handle = {
  0,
  &control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_msgs, msg, DynamicInterfaceGroupValues)() {
  control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_msgs, msg, InterfaceValue)();
  if (!control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_type_support_handle.typesupport_identifier) {
    control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &control_msgs__msg__DynamicInterfaceGroupValues__rosidl_typesupport_introspection_c__DynamicInterfaceGroupValues_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
