// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from control_msgs:msg/MultiDOFStateStamped.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "control_msgs/msg/detail/multi_dof_state_stamped__rosidl_typesupport_introspection_c.h"
#include "control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "control_msgs/msg/detail/multi_dof_state_stamped__functions.h"
#include "control_msgs/msg/detail/multi_dof_state_stamped__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `dof_states`
#include "control_msgs/msg/single_dof_state.h"
// Member `dof_states`
#include "control_msgs/msg/detail/single_dof_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  control_msgs__msg__MultiDOFStateStamped__init(message_memory);
}

void control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_fini_function(void * message_memory)
{
  control_msgs__msg__MultiDOFStateStamped__fini(message_memory);
}

size_t control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__size_function__MultiDOFStateStamped__dof_states(
  const void * untyped_member)
{
  const control_msgs__msg__SingleDOFState__Sequence * member =
    (const control_msgs__msg__SingleDOFState__Sequence *)(untyped_member);
  return member->size;
}

const void * control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__get_const_function__MultiDOFStateStamped__dof_states(
  const void * untyped_member, size_t index)
{
  const control_msgs__msg__SingleDOFState__Sequence * member =
    (const control_msgs__msg__SingleDOFState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__get_function__MultiDOFStateStamped__dof_states(
  void * untyped_member, size_t index)
{
  control_msgs__msg__SingleDOFState__Sequence * member =
    (control_msgs__msg__SingleDOFState__Sequence *)(untyped_member);
  return &member->data[index];
}

void control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__fetch_function__MultiDOFStateStamped__dof_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const control_msgs__msg__SingleDOFState * item =
    ((const control_msgs__msg__SingleDOFState *)
    control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__get_const_function__MultiDOFStateStamped__dof_states(untyped_member, index));
  control_msgs__msg__SingleDOFState * value =
    (control_msgs__msg__SingleDOFState *)(untyped_value);
  *value = *item;
}

void control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__assign_function__MultiDOFStateStamped__dof_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  control_msgs__msg__SingleDOFState * item =
    ((control_msgs__msg__SingleDOFState *)
    control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__get_function__MultiDOFStateStamped__dof_states(untyped_member, index));
  const control_msgs__msg__SingleDOFState * value =
    (const control_msgs__msg__SingleDOFState *)(untyped_value);
  *item = *value;
}

bool control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__resize_function__MultiDOFStateStamped__dof_states(
  void * untyped_member, size_t size)
{
  control_msgs__msg__SingleDOFState__Sequence * member =
    (control_msgs__msg__SingleDOFState__Sequence *)(untyped_member);
  control_msgs__msg__SingleDOFState__Sequence__fini(member);
  return control_msgs__msg__SingleDOFState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__MultiDOFStateStamped, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dof_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(control_msgs__msg__MultiDOFStateStamped, dof_states),  // bytes offset in struct
    NULL,  // default value
    control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__size_function__MultiDOFStateStamped__dof_states,  // size() function pointer
    control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__get_const_function__MultiDOFStateStamped__dof_states,  // get_const(index) function pointer
    control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__get_function__MultiDOFStateStamped__dof_states,  // get(index) function pointer
    control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__fetch_function__MultiDOFStateStamped__dof_states,  // fetch(index, &value) function pointer
    control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__assign_function__MultiDOFStateStamped__dof_states,  // assign(index, value) function pointer
    control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__resize_function__MultiDOFStateStamped__dof_states  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_members = {
  "control_msgs__msg",  // message namespace
  "MultiDOFStateStamped",  // message name
  2,  // number of fields
  sizeof(control_msgs__msg__MultiDOFStateStamped),
  control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_member_array,  // message members
  control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_type_support_handle = {
  0,
  &control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_msgs, msg, MultiDOFStateStamped)() {
  control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, control_msgs, msg, SingleDOFState)();
  if (!control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_type_support_handle.typesupport_identifier) {
    control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &control_msgs__msg__MultiDOFStateStamped__rosidl_typesupport_introspection_c__MultiDOFStateStamped_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
