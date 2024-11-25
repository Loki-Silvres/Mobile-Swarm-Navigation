// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from control_msgs:msg/DynamicInterfaceGroupValues.idl
// generated code does not contain a copyright notice
#include "control_msgs/msg/detail/dynamic_interface_group_values__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `interface_groups`
#include "rosidl_runtime_c/string_functions.h"
// Member `interface_values`
#include "control_msgs/msg/detail/interface_value__functions.h"

bool
control_msgs__msg__DynamicInterfaceGroupValues__init(control_msgs__msg__DynamicInterfaceGroupValues * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    control_msgs__msg__DynamicInterfaceGroupValues__fini(msg);
    return false;
  }
  // interface_groups
  if (!rosidl_runtime_c__String__Sequence__init(&msg->interface_groups, 0)) {
    control_msgs__msg__DynamicInterfaceGroupValues__fini(msg);
    return false;
  }
  // interface_values
  if (!control_msgs__msg__InterfaceValue__Sequence__init(&msg->interface_values, 0)) {
    control_msgs__msg__DynamicInterfaceGroupValues__fini(msg);
    return false;
  }
  return true;
}

void
control_msgs__msg__DynamicInterfaceGroupValues__fini(control_msgs__msg__DynamicInterfaceGroupValues * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // interface_groups
  rosidl_runtime_c__String__Sequence__fini(&msg->interface_groups);
  // interface_values
  control_msgs__msg__InterfaceValue__Sequence__fini(&msg->interface_values);
}

bool
control_msgs__msg__DynamicInterfaceGroupValues__are_equal(const control_msgs__msg__DynamicInterfaceGroupValues * lhs, const control_msgs__msg__DynamicInterfaceGroupValues * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // interface_groups
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->interface_groups), &(rhs->interface_groups)))
  {
    return false;
  }
  // interface_values
  if (!control_msgs__msg__InterfaceValue__Sequence__are_equal(
      &(lhs->interface_values), &(rhs->interface_values)))
  {
    return false;
  }
  return true;
}

bool
control_msgs__msg__DynamicInterfaceGroupValues__copy(
  const control_msgs__msg__DynamicInterfaceGroupValues * input,
  control_msgs__msg__DynamicInterfaceGroupValues * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // interface_groups
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->interface_groups), &(output->interface_groups)))
  {
    return false;
  }
  // interface_values
  if (!control_msgs__msg__InterfaceValue__Sequence__copy(
      &(input->interface_values), &(output->interface_values)))
  {
    return false;
  }
  return true;
}

control_msgs__msg__DynamicInterfaceGroupValues *
control_msgs__msg__DynamicInterfaceGroupValues__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__DynamicInterfaceGroupValues * msg = (control_msgs__msg__DynamicInterfaceGroupValues *)allocator.allocate(sizeof(control_msgs__msg__DynamicInterfaceGroupValues), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(control_msgs__msg__DynamicInterfaceGroupValues));
  bool success = control_msgs__msg__DynamicInterfaceGroupValues__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
control_msgs__msg__DynamicInterfaceGroupValues__destroy(control_msgs__msg__DynamicInterfaceGroupValues * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    control_msgs__msg__DynamicInterfaceGroupValues__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__init(control_msgs__msg__DynamicInterfaceGroupValues__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__DynamicInterfaceGroupValues * data = NULL;

  if (size) {
    data = (control_msgs__msg__DynamicInterfaceGroupValues *)allocator.zero_allocate(size, sizeof(control_msgs__msg__DynamicInterfaceGroupValues), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = control_msgs__msg__DynamicInterfaceGroupValues__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        control_msgs__msg__DynamicInterfaceGroupValues__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__fini(control_msgs__msg__DynamicInterfaceGroupValues__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      control_msgs__msg__DynamicInterfaceGroupValues__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

control_msgs__msg__DynamicInterfaceGroupValues__Sequence *
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__DynamicInterfaceGroupValues__Sequence * array = (control_msgs__msg__DynamicInterfaceGroupValues__Sequence *)allocator.allocate(sizeof(control_msgs__msg__DynamicInterfaceGroupValues__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = control_msgs__msg__DynamicInterfaceGroupValues__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__destroy(control_msgs__msg__DynamicInterfaceGroupValues__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    control_msgs__msg__DynamicInterfaceGroupValues__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__are_equal(const control_msgs__msg__DynamicInterfaceGroupValues__Sequence * lhs, const control_msgs__msg__DynamicInterfaceGroupValues__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!control_msgs__msg__DynamicInterfaceGroupValues__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
control_msgs__msg__DynamicInterfaceGroupValues__Sequence__copy(
  const control_msgs__msg__DynamicInterfaceGroupValues__Sequence * input,
  control_msgs__msg__DynamicInterfaceGroupValues__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(control_msgs__msg__DynamicInterfaceGroupValues);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    control_msgs__msg__DynamicInterfaceGroupValues * data =
      (control_msgs__msg__DynamicInterfaceGroupValues *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!control_msgs__msg__DynamicInterfaceGroupValues__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          control_msgs__msg__DynamicInterfaceGroupValues__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!control_msgs__msg__DynamicInterfaceGroupValues__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
