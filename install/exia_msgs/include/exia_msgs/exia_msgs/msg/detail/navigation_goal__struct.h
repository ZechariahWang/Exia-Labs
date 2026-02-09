// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from exia_msgs:msg/NavigationGoal.idl
// generated code does not contain a copyright notice

#ifndef EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__STRUCT_H_
#define EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'coord_type'
// Member 'lat_dms'
// Member 'lon_dms'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/NavigationGoal in the package exia_msgs.
typedef struct exia_msgs__msg__NavigationGoal
{
  rosidl_runtime_c__String coord_type;
  double x;
  double y;
  double lat;
  double lon;
  rosidl_runtime_c__String lat_dms;
  rosidl_runtime_c__String lon_dms;
  double origin_lat;
  double origin_lon;
} exia_msgs__msg__NavigationGoal;

// Struct for a sequence of exia_msgs__msg__NavigationGoal.
typedef struct exia_msgs__msg__NavigationGoal__Sequence
{
  exia_msgs__msg__NavigationGoal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} exia_msgs__msg__NavigationGoal__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__STRUCT_H_
