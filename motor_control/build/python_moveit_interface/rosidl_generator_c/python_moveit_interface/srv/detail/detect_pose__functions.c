// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from python_moveit_interface:srv/DetectPose.idl
// generated code does not contain a copyright notice
#include "python_moveit_interface/srv/detail/detect_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `target_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
python_moveit_interface__srv__DetectPose_Request__init(python_moveit_interface__srv__DetectPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__Pose__init(&msg->target_pose)) {
    python_moveit_interface__srv__DetectPose_Request__fini(msg);
    return false;
  }
  return true;
}

void
python_moveit_interface__srv__DetectPose_Request__fini(python_moveit_interface__srv__DetectPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // target_pose
  geometry_msgs__msg__Pose__fini(&msg->target_pose);
}

bool
python_moveit_interface__srv__DetectPose_Request__are_equal(const python_moveit_interface__srv__DetectPose_Request * lhs, const python_moveit_interface__srv__DetectPose_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->target_pose), &(rhs->target_pose)))
  {
    return false;
  }
  return true;
}

bool
python_moveit_interface__srv__DetectPose_Request__copy(
  const python_moveit_interface__srv__DetectPose_Request * input,
  python_moveit_interface__srv__DetectPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->target_pose), &(output->target_pose)))
  {
    return false;
  }
  return true;
}

python_moveit_interface__srv__DetectPose_Request *
python_moveit_interface__srv__DetectPose_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  python_moveit_interface__srv__DetectPose_Request * msg = (python_moveit_interface__srv__DetectPose_Request *)allocator.allocate(sizeof(python_moveit_interface__srv__DetectPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(python_moveit_interface__srv__DetectPose_Request));
  bool success = python_moveit_interface__srv__DetectPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
python_moveit_interface__srv__DetectPose_Request__destroy(python_moveit_interface__srv__DetectPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    python_moveit_interface__srv__DetectPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
python_moveit_interface__srv__DetectPose_Request__Sequence__init(python_moveit_interface__srv__DetectPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  python_moveit_interface__srv__DetectPose_Request * data = NULL;

  if (size) {
    data = (python_moveit_interface__srv__DetectPose_Request *)allocator.zero_allocate(size, sizeof(python_moveit_interface__srv__DetectPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = python_moveit_interface__srv__DetectPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        python_moveit_interface__srv__DetectPose_Request__fini(&data[i - 1]);
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
python_moveit_interface__srv__DetectPose_Request__Sequence__fini(python_moveit_interface__srv__DetectPose_Request__Sequence * array)
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
      python_moveit_interface__srv__DetectPose_Request__fini(&array->data[i]);
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

python_moveit_interface__srv__DetectPose_Request__Sequence *
python_moveit_interface__srv__DetectPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  python_moveit_interface__srv__DetectPose_Request__Sequence * array = (python_moveit_interface__srv__DetectPose_Request__Sequence *)allocator.allocate(sizeof(python_moveit_interface__srv__DetectPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = python_moveit_interface__srv__DetectPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
python_moveit_interface__srv__DetectPose_Request__Sequence__destroy(python_moveit_interface__srv__DetectPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    python_moveit_interface__srv__DetectPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
python_moveit_interface__srv__DetectPose_Request__Sequence__are_equal(const python_moveit_interface__srv__DetectPose_Request__Sequence * lhs, const python_moveit_interface__srv__DetectPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!python_moveit_interface__srv__DetectPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
python_moveit_interface__srv__DetectPose_Request__Sequence__copy(
  const python_moveit_interface__srv__DetectPose_Request__Sequence * input,
  python_moveit_interface__srv__DetectPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(python_moveit_interface__srv__DetectPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    python_moveit_interface__srv__DetectPose_Request * data =
      (python_moveit_interface__srv__DetectPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!python_moveit_interface__srv__DetectPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          python_moveit_interface__srv__DetectPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!python_moveit_interface__srv__DetectPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
python_moveit_interface__srv__DetectPose_Response__init(python_moveit_interface__srv__DetectPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    python_moveit_interface__srv__DetectPose_Response__fini(msg);
    return false;
  }
  return true;
}

void
python_moveit_interface__srv__DetectPose_Response__fini(python_moveit_interface__srv__DetectPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
python_moveit_interface__srv__DetectPose_Response__are_equal(const python_moveit_interface__srv__DetectPose_Response * lhs, const python_moveit_interface__srv__DetectPose_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
python_moveit_interface__srv__DetectPose_Response__copy(
  const python_moveit_interface__srv__DetectPose_Response * input,
  python_moveit_interface__srv__DetectPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

python_moveit_interface__srv__DetectPose_Response *
python_moveit_interface__srv__DetectPose_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  python_moveit_interface__srv__DetectPose_Response * msg = (python_moveit_interface__srv__DetectPose_Response *)allocator.allocate(sizeof(python_moveit_interface__srv__DetectPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(python_moveit_interface__srv__DetectPose_Response));
  bool success = python_moveit_interface__srv__DetectPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
python_moveit_interface__srv__DetectPose_Response__destroy(python_moveit_interface__srv__DetectPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    python_moveit_interface__srv__DetectPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
python_moveit_interface__srv__DetectPose_Response__Sequence__init(python_moveit_interface__srv__DetectPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  python_moveit_interface__srv__DetectPose_Response * data = NULL;

  if (size) {
    data = (python_moveit_interface__srv__DetectPose_Response *)allocator.zero_allocate(size, sizeof(python_moveit_interface__srv__DetectPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = python_moveit_interface__srv__DetectPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        python_moveit_interface__srv__DetectPose_Response__fini(&data[i - 1]);
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
python_moveit_interface__srv__DetectPose_Response__Sequence__fini(python_moveit_interface__srv__DetectPose_Response__Sequence * array)
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
      python_moveit_interface__srv__DetectPose_Response__fini(&array->data[i]);
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

python_moveit_interface__srv__DetectPose_Response__Sequence *
python_moveit_interface__srv__DetectPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  python_moveit_interface__srv__DetectPose_Response__Sequence * array = (python_moveit_interface__srv__DetectPose_Response__Sequence *)allocator.allocate(sizeof(python_moveit_interface__srv__DetectPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = python_moveit_interface__srv__DetectPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
python_moveit_interface__srv__DetectPose_Response__Sequence__destroy(python_moveit_interface__srv__DetectPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    python_moveit_interface__srv__DetectPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
python_moveit_interface__srv__DetectPose_Response__Sequence__are_equal(const python_moveit_interface__srv__DetectPose_Response__Sequence * lhs, const python_moveit_interface__srv__DetectPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!python_moveit_interface__srv__DetectPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
python_moveit_interface__srv__DetectPose_Response__Sequence__copy(
  const python_moveit_interface__srv__DetectPose_Response__Sequence * input,
  python_moveit_interface__srv__DetectPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(python_moveit_interface__srv__DetectPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    python_moveit_interface__srv__DetectPose_Response * data =
      (python_moveit_interface__srv__DetectPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!python_moveit_interface__srv__DetectPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          python_moveit_interface__srv__DetectPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!python_moveit_interface__srv__DetectPose_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
