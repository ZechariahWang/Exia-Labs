// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from exia_msgs:msg/NavigationGoal.idl
// generated code does not contain a copyright notice
#include "exia_msgs/msg/detail/navigation_goal__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "exia_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "exia_msgs/msg/detail/navigation_goal__struct.h"
#include "exia_msgs/msg/detail/navigation_goal__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // coord_type, lat_dms, lon_dms, move_type
#include "rosidl_runtime_c/string_functions.h"  // coord_type, lat_dms, lon_dms, move_type

// forward declare type support functions


using _NavigationGoal__ros_msg_type = exia_msgs__msg__NavigationGoal;

static bool _NavigationGoal__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _NavigationGoal__ros_msg_type * ros_message = static_cast<const _NavigationGoal__ros_msg_type *>(untyped_ros_message);
  // Field name: coord_type
  {
    const rosidl_runtime_c__String * str = &ros_message->coord_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: x
  {
    cdr << ros_message->x;
  }

  // Field name: y
  {
    cdr << ros_message->y;
  }

  // Field name: lat
  {
    cdr << ros_message->lat;
  }

  // Field name: lon
  {
    cdr << ros_message->lon;
  }

  // Field name: lat_dms
  {
    const rosidl_runtime_c__String * str = &ros_message->lat_dms;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: lon_dms
  {
    const rosidl_runtime_c__String * str = &ros_message->lon_dms;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: origin_lat
  {
    cdr << ros_message->origin_lat;
  }

  // Field name: origin_lon
  {
    cdr << ros_message->origin_lon;
  }

  // Field name: direct
  {
    cdr << (ros_message->direct ? true : false);
  }

  // Field name: move_type
  {
    const rosidl_runtime_c__String * str = &ros_message->move_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: move_value
  {
    cdr << ros_message->move_value;
  }

  // Field name: move_speed
  {
    cdr << ros_message->move_speed;
  }

  return true;
}

static bool _NavigationGoal__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _NavigationGoal__ros_msg_type * ros_message = static_cast<_NavigationGoal__ros_msg_type *>(untyped_ros_message);
  // Field name: coord_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->coord_type.data) {
      rosidl_runtime_c__String__init(&ros_message->coord_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->coord_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'coord_type'\n");
      return false;
    }
  }

  // Field name: x
  {
    cdr >> ros_message->x;
  }

  // Field name: y
  {
    cdr >> ros_message->y;
  }

  // Field name: lat
  {
    cdr >> ros_message->lat;
  }

  // Field name: lon
  {
    cdr >> ros_message->lon;
  }

  // Field name: lat_dms
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->lat_dms.data) {
      rosidl_runtime_c__String__init(&ros_message->lat_dms);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->lat_dms,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'lat_dms'\n");
      return false;
    }
  }

  // Field name: lon_dms
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->lon_dms.data) {
      rosidl_runtime_c__String__init(&ros_message->lon_dms);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->lon_dms,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'lon_dms'\n");
      return false;
    }
  }

  // Field name: origin_lat
  {
    cdr >> ros_message->origin_lat;
  }

  // Field name: origin_lon
  {
    cdr >> ros_message->origin_lon;
  }

  // Field name: direct
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->direct = tmp ? true : false;
  }

  // Field name: move_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->move_type.data) {
      rosidl_runtime_c__String__init(&ros_message->move_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->move_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'move_type'\n");
      return false;
    }
  }

  // Field name: move_value
  {
    cdr >> ros_message->move_value;
  }

  // Field name: move_speed
  {
    cdr >> ros_message->move_speed;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_exia_msgs
size_t get_serialized_size_exia_msgs__msg__NavigationGoal(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _NavigationGoal__ros_msg_type * ros_message = static_cast<const _NavigationGoal__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name coord_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->coord_type.size + 1);
  // field.name x
  {
    size_t item_size = sizeof(ros_message->x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y
  {
    size_t item_size = sizeof(ros_message->y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lat
  {
    size_t item_size = sizeof(ros_message->lat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lon
  {
    size_t item_size = sizeof(ros_message->lon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lat_dms
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->lat_dms.size + 1);
  // field.name lon_dms
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->lon_dms.size + 1);
  // field.name origin_lat
  {
    size_t item_size = sizeof(ros_message->origin_lat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name origin_lon
  {
    size_t item_size = sizeof(ros_message->origin_lon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name direct
  {
    size_t item_size = sizeof(ros_message->direct);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name move_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->move_type.size + 1);
  // field.name move_value
  {
    size_t item_size = sizeof(ros_message->move_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name move_speed
  {
    size_t item_size = sizeof(ros_message->move_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _NavigationGoal__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_exia_msgs__msg__NavigationGoal(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_exia_msgs
size_t max_serialized_size_exia_msgs__msg__NavigationGoal(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: coord_type
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: lat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: lon
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: lat_dms
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: lon_dms
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: origin_lat
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: origin_lon
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: direct
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: move_type
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: move_value
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: move_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = exia_msgs__msg__NavigationGoal;
    is_plain =
      (
      offsetof(DataType, move_speed) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _NavigationGoal__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_exia_msgs__msg__NavigationGoal(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_NavigationGoal = {
  "exia_msgs::msg",
  "NavigationGoal",
  _NavigationGoal__cdr_serialize,
  _NavigationGoal__cdr_deserialize,
  _NavigationGoal__get_serialized_size,
  _NavigationGoal__max_serialized_size
};

static rosidl_message_type_support_t _NavigationGoal__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_NavigationGoal,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, exia_msgs, msg, NavigationGoal)() {
  return &_NavigationGoal__type_support;
}

#if defined(__cplusplus)
}
#endif
