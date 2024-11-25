// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from control_msgs:msg/SingleDOFStateStamped.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "control_msgs/msg/detail/single_dof_state_stamped__rosidl_typesupport_introspection_c.h"
#include "control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "control_msgs/msg/detail/single_dof_state_stamped__functions.h"
#include "control_msgs/msg/detail/single_dof_state_stamped__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `dof_state`
#include "control_msgs/msg/single_dof_state.h"
// Member `dof_state`
#include "control_msgs/msg/detail/single_dof_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  control_msgs__msg__SingleDOFStateStamped__init(message_memory);
}

void control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_fini_function(void * message_memory)
{
  control_msgs__msg__SingleDOFStateStamped__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__SingleDOFStateStamped, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dof_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__SingleDOFStateStamped, dof_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_members = {
  "control_msgs__msg",  // message namespace
  "SingleDOFStateStamped",  // message name
  2,  // number of fields
  sizeof(control_msgs__msg__SingleDOFStateStamped),
  control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_member_array,  // message members
  control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_type_support_handle = {
  0,
  &control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_msgs, msg, SingleDOFStateStamped)() {
  control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_msgs, msg, SingleDOFState)();
  if (!control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_type_support_handle.typesupport_identifier) {
    control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &control_msgs__msg__SingleDOFStateStamped__rosidl_typesupport_introspection_c__SingleDOFStateStamped_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
