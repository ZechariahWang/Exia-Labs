// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from exia_msgs:msg/NavigationGoal.idl
// generated code does not contain a copyright notice
#include "exia_msgs/msg/detail/navigation_goal__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `coord_type`
// Member `lat_dms`
// Member `lon_dms`
#include "rosidl_runtime_c/string_functions.h"

bool
exia_msgs__msg__NavigationGoal__init(exia_msgs__msg__NavigationGoal * msg)
{
  if (!msg) {
    return false;
  }
  // coord_type
  if (!rosidl_runtime_c__String__init(&msg->coord_type)) {
    exia_msgs__msg__NavigationGoal__fini(msg);
    return false;
  }
  // x
  // y
  // lat
  // lon
  // lat_dms
  if (!rosidl_runtime_c__String__init(&msg->lat_dms)) {
    exia_msgs__msg__NavigationGoal__fini(msg);
    return false;
  }
  // lon_dms
  if (!rosidl_runtime_c__String__init(&msg->lon_dms)) {
    exia_msgs__msg__NavigationGoal__fini(msg);
    return false;
  }
  // origin_lat
  // origin_lon
  // direct
  return true;
}

void
exia_msgs__msg__NavigationGoal__fini(exia_msgs__msg__NavigationGoal * msg)
{
  if (!msg) {
    return;
  }
  // coord_type
  rosidl_runtime_c__String__fini(&msg->coord_type);
  // x
  // y
  // lat
  // lon
  // lat_dms
  rosidl_runtime_c__String__fini(&msg->lat_dms);
  // lon_dms
  rosidl_runtime_c__String__fini(&msg->lon_dms);
  // origin_lat
  // origin_lon
  // direct
}

bool
exia_msgs__msg__NavigationGoal__are_equal(const exia_msgs__msg__NavigationGoal * lhs, const exia_msgs__msg__NavigationGoal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // coord_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->coord_type), &(rhs->coord_type)))
  {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // lat
  if (lhs->lat != rhs->lat) {
    return false;
  }
  // lon
  if (lhs->lon != rhs->lon) {
    return false;
  }
  // lat_dms
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->lat_dms), &(rhs->lat_dms)))
  {
    return false;
  }
  // lon_dms
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->lon_dms), &(rhs->lon_dms)))
  {
    return false;
  }
  // origin_lat
  if (lhs->origin_lat != rhs->origin_lat) {
    return false;
  }
  // origin_lon
  if (lhs->origin_lon != rhs->origin_lon) {
    return false;
  }
  // direct
  if (lhs->direct != rhs->direct) {
    return false;
  }
  return true;
}

bool
exia_msgs__msg__NavigationGoal__copy(
  const exia_msgs__msg__NavigationGoal * input,
  exia_msgs__msg__NavigationGoal * output)
{
  if (!input || !output) {
    return false;
  }
  // coord_type
  if (!rosidl_runtime_c__String__copy(
      &(input->coord_type), &(output->coord_type)))
  {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // lat
  output->lat = input->lat;
  // lon
  output->lon = input->lon;
  // lat_dms
  if (!rosidl_runtime_c__String__copy(
      &(input->lat_dms), &(output->lat_dms)))
  {
    return false;
  }
  // lon_dms
  if (!rosidl_runtime_c__String__copy(
      &(input->lon_dms), &(output->lon_dms)))
  {
    return false;
  }
  // origin_lat
  output->origin_lat = input->origin_lat;
  // origin_lon
  output->origin_lon = input->origin_lon;
  // direct
  output->direct = input->direct;
  return true;
}

exia_msgs__msg__NavigationGoal *
exia_msgs__msg__NavigationGoal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  exia_msgs__msg__NavigationGoal * msg = (exia_msgs__msg__NavigationGoal *)allocator.allocate(sizeof(exia_msgs__msg__NavigationGoal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(exia_msgs__msg__NavigationGoal));
  bool success = exia_msgs__msg__NavigationGoal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
exia_msgs__msg__NavigationGoal__destroy(exia_msgs__msg__NavigationGoal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    exia_msgs__msg__NavigationGoal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
exia_msgs__msg__NavigationGoal__Sequence__init(exia_msgs__msg__NavigationGoal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  exia_msgs__msg__NavigationGoal * data = NULL;

  if (size) {
    data = (exia_msgs__msg__NavigationGoal *)allocator.zero_allocate(size, sizeof(exia_msgs__msg__NavigationGoal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = exia_msgs__msg__NavigationGoal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        exia_msgs__msg__NavigationGoal__fini(&data[i - 1]);
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
exia_msgs__msg__NavigationGoal__Sequence__fini(exia_msgs__msg__NavigationGoal__Sequence * array)
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
      exia_msgs__msg__NavigationGoal__fini(&array->data[i]);
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

exia_msgs__msg__NavigationGoal__Sequence *
exia_msgs__msg__NavigationGoal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  exia_msgs__msg__NavigationGoal__Sequence * array = (exia_msgs__msg__NavigationGoal__Sequence *)allocator.allocate(sizeof(exia_msgs__msg__NavigationGoal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = exia_msgs__msg__NavigationGoal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
exia_msgs__msg__NavigationGoal__Sequence__destroy(exia_msgs__msg__NavigationGoal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    exia_msgs__msg__NavigationGoal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
exia_msgs__msg__NavigationGoal__Sequence__are_equal(const exia_msgs__msg__NavigationGoal__Sequence * lhs, const exia_msgs__msg__NavigationGoal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!exia_msgs__msg__NavigationGoal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
exia_msgs__msg__NavigationGoal__Sequence__copy(
  const exia_msgs__msg__NavigationGoal__Sequence * input,
  exia_msgs__msg__NavigationGoal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(exia_msgs__msg__NavigationGoal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    exia_msgs__msg__NavigationGoal * data =
      (exia_msgs__msg__NavigationGoal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!exia_msgs__msg__NavigationGoal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          exia_msgs__msg__NavigationGoal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!exia_msgs__msg__NavigationGoal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
