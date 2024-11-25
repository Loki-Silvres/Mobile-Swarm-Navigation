// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from control_msgs:msg/MultiDOFCommand.idl
// generated code does not contain a copyright notice
#include "control_msgs/msg/detail/multi_dof_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `dof_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `values`
// Member `values_dot`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
control_msgs__msg__MultiDOFCommand__init(control_msgs__msg__MultiDOFCommand * msg)
{
  if (!msg) {
    return false;
  }
  // dof_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->dof_names, 0)) {
    control_msgs__msg__MultiDOFCommand__fini(msg);
    return false;
  }
  // values
  if (!rosidl_runtime_c__double__Sequence__init(&msg->values, 0)) {
    control_msgs__msg__MultiDOFCommand__fini(msg);
    return false;
  }
  // values_dot
  if (!rosidl_runtime_c__double__Sequence__init(&msg->values_dot, 0)) {
    control_msgs__msg__MultiDOFCommand__fini(msg);
    return false;
  }
  return true;
}

void
control_msgs__msg__MultiDOFCommand__fini(control_msgs__msg__MultiDOFCommand * msg)
{
  if (!msg) {
    return;
  }
  // dof_names
  rosidl_runtime_c__String__Sequence__fini(&msg->dof_names);
  // values
  rosidl_runtime_c__double__Sequence__fini(&msg->values);
  // values_dot
  rosidl_runtime_c__double__Sequence__fini(&msg->values_dot);
}

bool
control_msgs__msg__MultiDOFCommand__are_equal(const control_msgs__msg__MultiDOFCommand * lhs, const control_msgs__msg__MultiDOFCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // dof_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->dof_names), &(rhs->dof_names)))
  {
    return false;
  }
  // values
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->values), &(rhs->values)))
  {
    return false;
  }
  // values_dot
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->values_dot), &(rhs->values_dot)))
  {
    return false;
  }
  return true;
}

bool
control_msgs__msg__MultiDOFCommand__copy(
  const control_msgs__msg__MultiDOFCommand * input,
  control_msgs__msg__MultiDOFCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // dof_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->dof_names), &(output->dof_names)))
  {
    return false;
  }
  // values
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->values), &(output->values)))
  {
    return false;
  }
  // values_dot
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->values_dot), &(output->values_dot)))
  {
    return false;
  }
  return true;
}

control_msgs__msg__MultiDOFCommand *
control_msgs__msg__MultiDOFCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__MultiDOFCommand * msg = (control_msgs__msg__MultiDOFCommand *)allocator.allocate(sizeof(control_msgs__msg__MultiDOFCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(control_msgs__msg__MultiDOFCommand));
  bool success = control_msgs__msg__MultiDOFCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
control_msgs__msg__MultiDOFCommand__destroy(control_msgs__msg__MultiDOFCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    control_msgs__msg__MultiDOFCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
control_msgs__msg__MultiDOFCommand__Sequence__init(control_msgs__msg__MultiDOFCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__MultiDOFCommand * data = NULL;

  if (size) {
    data = (control_msgs__msg__MultiDOFCommand *)allocator.zero_allocate(size, sizeof(control_msgs__msg__MultiDOFCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = control_msgs__msg__MultiDOFCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        control_msgs__msg__MultiDOFCommand__fini(&data[i - 1]);
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
control_msgs__msg__MultiDOFCommand__Sequence__fini(control_msgs__msg__MultiDOFCommand__Sequence * array)
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
      control_msgs__msg__MultiDOFCommand__fini(&array->data[i]);
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

control_msgs__msg__MultiDOFCommand__Sequence *
control_msgs__msg__MultiDOFCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__MultiDOFCommand__Sequence * array = (control_msgs__msg__MultiDOFCommand__Sequence *)allocator.allocate(sizeof(control_msgs__msg__MultiDOFCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = control_msgs__msg__MultiDOFCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
control_msgs__msg__MultiDOFCommand__Sequence__destroy(control_msgs__msg__MultiDOFCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    control_msgs__msg__MultiDOFCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
control_msgs__msg__MultiDOFCommand__Sequence__are_equal(const control_msgs__msg__MultiDOFCommand__Sequence * lhs, const control_msgs__msg__MultiDOFCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!control_msgs__msg__MultiDOFCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
control_msgs__msg__MultiDOFCommand__Sequence__copy(
  const control_msgs__msg__MultiDOFCommand__Sequence * input,
  control_msgs__msg__MultiDOFCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(control_msgs__msg__MultiDOFCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    control_msgs__msg__MultiDOFCommand * data =
      (control_msgs__msg__MultiDOFCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!control_msgs__msg__MultiDOFCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          control_msgs__msg__MultiDOFCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!control_msgs__msg__MultiDOFCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
