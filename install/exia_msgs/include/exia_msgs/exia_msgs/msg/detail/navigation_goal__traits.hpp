// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from exia_msgs:msg/NavigationGoal.idl
// generated code does not contain a copyright notice

#ifndef EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__TRAITS_HPP_
#define EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "exia_msgs/msg/detail/navigation_goal__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace exia_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const NavigationGoal & msg,
  std::ostream & out)
{
  out << "{";
  // member: coord_type
  {
    out << "coord_type: ";
    rosidl_generator_traits::value_to_yaml(msg.coord_type, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: lat
  {
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << ", ";
  }

  // member: lon
  {
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << ", ";
  }

  // member: lat_dms
  {
    out << "lat_dms: ";
    rosidl_generator_traits::value_to_yaml(msg.lat_dms, out);
    out << ", ";
  }

  // member: lon_dms
  {
    out << "lon_dms: ";
    rosidl_generator_traits::value_to_yaml(msg.lon_dms, out);
    out << ", ";
  }

  // member: origin_lat
  {
    out << "origin_lat: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_lat, out);
    out << ", ";
  }

  // member: origin_lon
  {
    out << "origin_lon: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_lon, out);
    out << ", ";
  }

  // member: direct
  {
    out << "direct: ";
    rosidl_generator_traits::value_to_yaml(msg.direct, out);
    out << ", ";
  }

  // member: move_type
  {
    out << "move_type: ";
    rosidl_generator_traits::value_to_yaml(msg.move_type, out);
    out << ", ";
  }

  // member: move_value
  {
    out << "move_value: ";
    rosidl_generator_traits::value_to_yaml(msg.move_value, out);
    out << ", ";
  }

  // member: move_speed
  {
    out << "move_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.move_speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigationGoal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: coord_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coord_type: ";
    rosidl_generator_traits::value_to_yaml(msg.coord_type, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << "\n";
  }

  // member: lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << "\n";
  }

  // member: lat_dms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat_dms: ";
    rosidl_generator_traits::value_to_yaml(msg.lat_dms, out);
    out << "\n";
  }

  // member: lon_dms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon_dms: ";
    rosidl_generator_traits::value_to_yaml(msg.lon_dms, out);
    out << "\n";
  }

  // member: origin_lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_lat: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_lat, out);
    out << "\n";
  }

  // member: origin_lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "origin_lon: ";
    rosidl_generator_traits::value_to_yaml(msg.origin_lon, out);
    out << "\n";
  }

  // member: direct
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direct: ";
    rosidl_generator_traits::value_to_yaml(msg.direct, out);
    out << "\n";
  }

  // member: move_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_type: ";
    rosidl_generator_traits::value_to_yaml(msg.move_type, out);
    out << "\n";
  }

  // member: move_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_value: ";
    rosidl_generator_traits::value_to_yaml(msg.move_value, out);
    out << "\n";
  }

  // member: move_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.move_speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigationGoal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace exia_msgs

namespace rosidl_generator_traits
{

[[deprecated("use exia_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const exia_msgs::msg::NavigationGoal & msg,
  std::ostream & out, size_t indentation = 0)
{
  exia_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use exia_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const exia_msgs::msg::NavigationGoal & msg)
{
  return exia_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<exia_msgs::msg::NavigationGoal>()
{
  return "exia_msgs::msg::NavigationGoal";
}

template<>
inline const char * name<exia_msgs::msg::NavigationGoal>()
{
  return "exia_msgs/msg/NavigationGoal";
}

template<>
struct has_fixed_size<exia_msgs::msg::NavigationGoal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<exia_msgs::msg::NavigationGoal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<exia_msgs::msg::NavigationGoal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__TRAITS_HPP_
