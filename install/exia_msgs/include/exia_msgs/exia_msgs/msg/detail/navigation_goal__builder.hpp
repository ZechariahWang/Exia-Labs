// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from exia_msgs:msg/NavigationGoal.idl
// generated code does not contain a copyright notice

#ifndef EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__BUILDER_HPP_
#define EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "exia_msgs/msg/detail/navigation_goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace exia_msgs
{

namespace msg
{

namespace builder
{

class Init_NavigationGoal_move_speed
{
public:
  explicit Init_NavigationGoal_move_speed(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  ::exia_msgs::msg::NavigationGoal move_speed(::exia_msgs::msg::NavigationGoal::_move_speed_type arg)
  {
    msg_.move_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_move_value
{
public:
  explicit Init_NavigationGoal_move_value(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_move_speed move_value(::exia_msgs::msg::NavigationGoal::_move_value_type arg)
  {
    msg_.move_value = std::move(arg);
    return Init_NavigationGoal_move_speed(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_move_type
{
public:
  explicit Init_NavigationGoal_move_type(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_move_value move_type(::exia_msgs::msg::NavigationGoal::_move_type_type arg)
  {
    msg_.move_type = std::move(arg);
    return Init_NavigationGoal_move_value(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_direct
{
public:
  explicit Init_NavigationGoal_direct(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_move_type direct(::exia_msgs::msg::NavigationGoal::_direct_type arg)
  {
    msg_.direct = std::move(arg);
    return Init_NavigationGoal_move_type(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_origin_lon
{
public:
  explicit Init_NavigationGoal_origin_lon(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_direct origin_lon(::exia_msgs::msg::NavigationGoal::_origin_lon_type arg)
  {
    msg_.origin_lon = std::move(arg);
    return Init_NavigationGoal_direct(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_origin_lat
{
public:
  explicit Init_NavigationGoal_origin_lat(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_origin_lon origin_lat(::exia_msgs::msg::NavigationGoal::_origin_lat_type arg)
  {
    msg_.origin_lat = std::move(arg);
    return Init_NavigationGoal_origin_lon(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_lon_dms
{
public:
  explicit Init_NavigationGoal_lon_dms(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_origin_lat lon_dms(::exia_msgs::msg::NavigationGoal::_lon_dms_type arg)
  {
    msg_.lon_dms = std::move(arg);
    return Init_NavigationGoal_origin_lat(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_lat_dms
{
public:
  explicit Init_NavigationGoal_lat_dms(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_lon_dms lat_dms(::exia_msgs::msg::NavigationGoal::_lat_dms_type arg)
  {
    msg_.lat_dms = std::move(arg);
    return Init_NavigationGoal_lon_dms(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_lon
{
public:
  explicit Init_NavigationGoal_lon(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_lat_dms lon(::exia_msgs::msg::NavigationGoal::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_NavigationGoal_lat_dms(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_lat
{
public:
  explicit Init_NavigationGoal_lat(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_lon lat(::exia_msgs::msg::NavigationGoal::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_NavigationGoal_lon(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_y
{
public:
  explicit Init_NavigationGoal_y(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_lat y(::exia_msgs::msg::NavigationGoal::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_NavigationGoal_lat(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_x
{
public:
  explicit Init_NavigationGoal_x(::exia_msgs::msg::NavigationGoal & msg)
  : msg_(msg)
  {}
  Init_NavigationGoal_y x(::exia_msgs::msg::NavigationGoal::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_NavigationGoal_y(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

class Init_NavigationGoal_coord_type
{
public:
  Init_NavigationGoal_coord_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigationGoal_x coord_type(::exia_msgs::msg::NavigationGoal::_coord_type_type arg)
  {
    msg_.coord_type = std::move(arg);
    return Init_NavigationGoal_x(msg_);
  }

private:
  ::exia_msgs::msg::NavigationGoal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::exia_msgs::msg::NavigationGoal>()
{
  return exia_msgs::msg::builder::Init_NavigationGoal_coord_type();
}

}  // namespace exia_msgs

#endif  // EXIA_MSGS__MSG__DETAIL__NAVIGATION_GOAL__BUILDER_HPP_
