// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jackal_msgs:msg/Power.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__POWER__BUILDER_HPP_
#define JACKAL_MSGS__MSG__DETAIL__POWER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jackal_msgs/msg/detail/power__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jackal_msgs
{

namespace msg
{

namespace builder
{

class Init_Power_measured_currents
{
public:
  explicit Init_Power_measured_currents(::jackal_msgs::msg::Power & msg)
  : msg_(msg)
  {}
  ::jackal_msgs::msg::Power measured_currents(::jackal_msgs::msg::Power::_measured_currents_type arg)
  {
    msg_.measured_currents = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jackal_msgs::msg::Power msg_;
};

class Init_Power_measured_voltages
{
public:
  explicit Init_Power_measured_voltages(::jackal_msgs::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_measured_currents measured_voltages(::jackal_msgs::msg::Power::_measured_voltages_type arg)
  {
    msg_.measured_voltages = std::move(arg);
    return Init_Power_measured_currents(msg_);
  }

private:
  ::jackal_msgs::msg::Power msg_;
};

class Init_Power_charging_complete
{
public:
  explicit Init_Power_charging_complete(::jackal_msgs::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_measured_voltages charging_complete(::jackal_msgs::msg::Power::_charging_complete_type arg)
  {
    msg_.charging_complete = std::move(arg);
    return Init_Power_measured_voltages(msg_);
  }

private:
  ::jackal_msgs::msg::Power msg_;
};

class Init_Power_power_12v_user_nominal
{
public:
  explicit Init_Power_power_12v_user_nominal(::jackal_msgs::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_charging_complete power_12v_user_nominal(::jackal_msgs::msg::Power::_power_12v_user_nominal_type arg)
  {
    msg_.power_12v_user_nominal = std::move(arg);
    return Init_Power_charging_complete(msg_);
  }

private:
  ::jackal_msgs::msg::Power msg_;
};

class Init_Power_battery_connected
{
public:
  explicit Init_Power_battery_connected(::jackal_msgs::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_power_12v_user_nominal battery_connected(::jackal_msgs::msg::Power::_battery_connected_type arg)
  {
    msg_.battery_connected = std::move(arg);
    return Init_Power_power_12v_user_nominal(msg_);
  }

private:
  ::jackal_msgs::msg::Power msg_;
};

class Init_Power_shore_power_connected
{
public:
  explicit Init_Power_shore_power_connected(::jackal_msgs::msg::Power & msg)
  : msg_(msg)
  {}
  Init_Power_battery_connected shore_power_connected(::jackal_msgs::msg::Power::_shore_power_connected_type arg)
  {
    msg_.shore_power_connected = std::move(arg);
    return Init_Power_battery_connected(msg_);
  }

private:
  ::jackal_msgs::msg::Power msg_;
};

class Init_Power_header
{
public:
  Init_Power_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Power_shore_power_connected header(::jackal_msgs::msg::Power::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Power_shore_power_connected(msg_);
  }

private:
  ::jackal_msgs::msg::Power msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jackal_msgs::msg::Power>()
{
  return jackal_msgs::msg::builder::Init_Power_header();
}

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__MSG__DETAIL__POWER__BUILDER_HPP_
