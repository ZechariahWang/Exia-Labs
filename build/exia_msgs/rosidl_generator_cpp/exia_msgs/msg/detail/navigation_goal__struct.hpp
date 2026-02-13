// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from exia_msgs:msg/NavigationGoal.idl
// generated code does not contain a copyright notice

#ifndef EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__STRUCT_HPP_
#define EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__exia_msgs__msg__NavigationGoal __attribute__((deprecated))
#else
# define DEPRECATED__exia_msgs__msg__NavigationGoal __declspec(deprecated)
#endif

namespace exia_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavigationGoal_
{
  using Type = NavigationGoal_<ContainerAllocator>;

  explicit NavigationGoal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->coord_type = "";
      this->x = 0.0;
      this->y = 0.0;
      this->lat = 0.0;
      this->lon = 0.0;
      this->lat_dms = "";
      this->lon_dms = "";
      this->origin_lat = 0.0;
      this->origin_lon = 0.0;
      this->direct = false;
      this->move_type = "";
      this->move_value = 0.0;
      this->move_speed = 0.0;
    }
  }

  explicit NavigationGoal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : coord_type(_alloc),
    lat_dms(_alloc),
    lon_dms(_alloc),
    move_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->coord_type = "";
      this->x = 0.0;
      this->y = 0.0;
      this->lat = 0.0;
      this->lon = 0.0;
      this->lat_dms = "";
      this->lon_dms = "";
      this->origin_lat = 0.0;
      this->origin_lon = 0.0;
      this->direct = false;
      this->move_type = "";
      this->move_value = 0.0;
      this->move_speed = 0.0;
    }
  }

  // field types and members
  using _coord_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _coord_type_type coord_type;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _lat_type =
    double;
  _lat_type lat;
  using _lon_type =
    double;
  _lon_type lon;
  using _lat_dms_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _lat_dms_type lat_dms;
  using _lon_dms_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _lon_dms_type lon_dms;
  using _origin_lat_type =
    double;
  _origin_lat_type origin_lat;
  using _origin_lon_type =
    double;
  _origin_lon_type origin_lon;
  using _direct_type =
    bool;
  _direct_type direct;
  using _move_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _move_type_type move_type;
  using _move_value_type =
    double;
  _move_value_type move_value;
  using _move_speed_type =
    double;
  _move_speed_type move_speed;

  // setters for named parameter idiom
  Type & set__coord_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->coord_type = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__lat(
    const double & _arg)
  {
    this->lat = _arg;
    return *this;
  }
  Type & set__lon(
    const double & _arg)
  {
    this->lon = _arg;
    return *this;
  }
  Type & set__lat_dms(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->lat_dms = _arg;
    return *this;
  }
  Type & set__lon_dms(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->lon_dms = _arg;
    return *this;
  }
  Type & set__origin_lat(
    const double & _arg)
  {
    this->origin_lat = _arg;
    return *this;
  }
  Type & set__origin_lon(
    const double & _arg)
  {
    this->origin_lon = _arg;
    return *this;
  }
  Type & set__direct(
    const bool & _arg)
  {
    this->direct = _arg;
    return *this;
  }
  Type & set__move_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->move_type = _arg;
    return *this;
  }
  Type & set__move_value(
    const double & _arg)
  {
    this->move_value = _arg;
    return *this;
  }
  Type & set__move_speed(
    const double & _arg)
  {
    this->move_speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    exia_msgs::msg::NavigationGoal_<ContainerAllocator> *;
  using ConstRawPtr =
    const exia_msgs::msg::NavigationGoal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<exia_msgs::msg::NavigationGoal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<exia_msgs::msg::NavigationGoal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      exia_msgs::msg::NavigationGoal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<exia_msgs::msg::NavigationGoal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      exia_msgs::msg::NavigationGoal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<exia_msgs::msg::NavigationGoal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<exia_msgs::msg::NavigationGoal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<exia_msgs::msg::NavigationGoal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__exia_msgs__msg__NavigationGoal
    std::shared_ptr<exia_msgs::msg::NavigationGoal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__exia_msgs__msg__NavigationGoal
    std::shared_ptr<exia_msgs::msg::NavigationGoal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigationGoal_ & other) const
  {
    if (this->coord_type != other.coord_type) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->lat != other.lat) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    if (this->lat_dms != other.lat_dms) {
      return false;
    }
    if (this->lon_dms != other.lon_dms) {
      return false;
    }
    if (this->origin_lat != other.origin_lat) {
      return false;
    }
    if (this->origin_lon != other.origin_lon) {
      return false;
    }
    if (this->direct != other.direct) {
      return false;
    }
    if (this->move_type != other.move_type) {
      return false;
    }
    if (this->move_value != other.move_value) {
      return false;
    }
    if (this->move_speed != other.move_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigationGoal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigationGoal_

// alias to use template instance with default allocator
using NavigationGoal =
  exia_msgs::msg::NavigationGoal_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace exia_msgs

#endif  // EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__STRUCT_HPP_
