// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from control_msgs:msg/SingleDOFStateStamped.idl
// generated code does not contain a copyright notice
#include "control_msgs/msg/detail/single_dof_state_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `dof_state`
#include "control_msgs/msg/detail/single_dof_state__functions.h"

bool
control_msgs__msg__SingleDOFStateStamped__init(control_msgs__msg__SingleDOFStateStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    control_msgs__msg__SingleDOFStateStamped__fini(msg);
    return false;
  }
  // dof_state
  if (!control_msgs__msg__SingleDOFState__init(&msg->dof_state)) {
    control_msgs__msg__SingleDOFStateStamped__fini(msg);
    return false;
  }
  return true;
}

void
control_msgs__msg__SingleDOFStateStamped__fini(control_msgs__msg__SingleDOFStateStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // dof_state
  control_msgs__msg__SingleDOFState__fini(&msg->dof_state);
}

bool
control_msgs__msg__SingleDOFStateStamped__are_equal(const control_msgs__msg__SingleDOFStateStamped * lhs, const control_msgs__msg__SingleDOFStateStamped * rhs)
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
  // dof_state
  if (!control_msgs__msg__SingleDOFState__are_equal(
      &(lhs->dof_state), &(rhs->dof_state)))
  {
    return false;
  }
  return true;
}

bool
control_msgs__msg__SingleDOFStateStamped__copy(
  const control_msgs__msg__SingleDOFStateStamped * input,
  control_msgs__msg__SingleDOFStateStamped * output)
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
  // dof_state
  if (!control_msgs__msg__SingleDOFState__copy(
      &(input->dof_state), &(output->dof_state)))
  {
    return false;
  }
  return true;
}

control_msgs__msg__SingleDOFStateStamped *
control_msgs__msg__SingleDOFStateStamped__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__SingleDOFStateStamped * msg = (control_msgs__msg__SingleDOFStateStamped *)allocator.allocate(sizeof(control_msgs__msg__SingleDOFStateStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(control_msgs__msg__SingleDOFStateStamped));
  bool success = control_msgs__msg__SingleDOFStateStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
control_msgs__msg__SingleDOFStateStamped__destroy(control_msgs__msg__SingleDOFStateStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    control_msgs__msg__SingleDOFStateStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
control_msgs__msg__SingleDOFStateStamped__Sequence__init(control_msgs__msg__SingleDOFStateStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__SingleDOFStateStamped * data = NULL;

  if (size) {
    data = (control_msgs__msg__SingleDOFStateStamped *)allocator.zero_allocate(size, sizeof(control_msgs__msg__SingleDOFStateStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = control_msgs__msg__SingleDOFStateStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        control_msgs__msg__SingleDOFStateStamped__fini(&data[i - 1]);
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
control_msgs__msg__SingleDOFStateStamped__Sequence__fini(control_msgs__msg__SingleDOFStateStamped__Sequence * array)
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
      control_msgs__msg__SingleDOFStateStamped__fini(&array->data[i]);
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

control_msgs__msg__SingleDOFStateStamped__Sequence *
control_msgs__msg__SingleDOFStateStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  control_msgs__msg__SingleDOFStateStamped__Sequence * array = (control_msgs__msg__SingleDOFStateStamped__Sequence *)allocator.allocate(sizeof(control_msgs__msg__SingleDOFStateStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = control_msgs__msg__SingleDOFStateStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
control_msgs__msg__SingleDOFStateStamped__Sequence__destroy(control_msgs__msg__SingleDOFStateStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    control_msgs__msg__SingleDOFStateStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
control_msgs__msg__SingleDOFStateStamped__Sequence__are_equal(const control_msgs__msg__SingleDOFStateStamped__Sequence * lhs, const control_msgs__msg__SingleDOFStateStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!control_msgs__msg__SingleDOFStateStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
control_msgs__msg__SingleDOFStateStamped__Sequence__copy(
  const control_msgs__msg__SingleDOFStateStamped__Sequence * input,
  control_msgs__msg__SingleDOFStateStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(control_msgs__msg__SingleDOFStateStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    control_msgs__msg__SingleDOFStateStamped * data =
      (control_msgs__msg__SingleDOFStateStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!control_msgs__msg__SingleDOFStateStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          control_msgs__msg__SingleDOFStateStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!control_msgs__msg__SingleDOFStateStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
