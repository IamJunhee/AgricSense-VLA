// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jackal_msgs:msg/DriveFeedback.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__BUILDER_HPP_
#define JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jackal_msgs/msg/detail/drive_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jackal_msgs
{

namespace msg
{

namespace builder
{

class Init_DriveFeedback_driver_fault
{
public:
  explicit Init_DriveFeedback_driver_fault(::jackal_msgs::msg::DriveFeedback & msg)
  : msg_(msg)
  {}
  ::jackal_msgs::msg::DriveFeedback driver_fault(::jackal_msgs::msg::DriveFeedback::_driver_fault_type arg)
  {
    msg_.driver_fault = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jackal_msgs::msg::DriveFeedback msg_;
};

class Init_DriveFeedback_measured_travel
{
public:
  explicit Init_DriveFeedback_measured_travel(::jackal_msgs::msg::DriveFeedback & msg)
  : msg_(msg)
  {}
  Init_DriveFeedback_driver_fault measured_travel(::jackal_msgs::msg::DriveFeedback::_measured_travel_type arg)
  {
    msg_.measured_travel = std::move(arg);
    return Init_DriveFeedback_driver_fault(msg_);
  }

private:
  ::jackal_msgs::msg::DriveFeedback msg_;
};

class Init_DriveFeedback_measured_velocity
{
public:
  explicit Init_DriveFeedback_measured_velocity(::jackal_msgs::msg::DriveFeedback & msg)
  : msg_(msg)
  {}
  Init_DriveFeedback_measured_travel measured_velocity(::jackal_msgs::msg::DriveFeedback::_measured_velocity_type arg)
  {
    msg_.measured_velocity = std::move(arg);
    return Init_DriveFeedback_measured_travel(msg_);
  }

private:
  ::jackal_msgs::msg::DriveFeedback msg_;
};

class Init_DriveFeedback_motor_temperature
{
public:
  explicit Init_DriveFeedback_motor_temperature(::jackal_msgs::msg::DriveFeedback & msg)
  : msg_(msg)
  {}
  Init_DriveFeedback_measured_velocity motor_temperature(::jackal_msgs::msg::DriveFeedback::_motor_temperature_type arg)
  {
    msg_.motor_temperature = std::move(arg);
    return Init_DriveFeedback_measured_velocity(msg_);
  }

private:
  ::jackal_msgs::msg::DriveFeedback msg_;
};

class Init_DriveFeedback_bridge_temperature
{
public:
  explicit Init_DriveFeedback_bridge_temperature(::jackal_msgs::msg::DriveFeedback & msg)
  : msg_(msg)
  {}
  Init_DriveFeedback_motor_temperature bridge_temperature(::jackal_msgs::msg::DriveFeedback::_bridge_temperature_type arg)
  {
    msg_.bridge_temperature = std::move(arg);
    return Init_DriveFeedback_motor_temperature(msg_);
  }

private:
  ::jackal_msgs::msg::DriveFeedback msg_;
};

class Init_DriveFeedback_duty_cycle
{
public:
  explicit Init_DriveFeedback_duty_cycle(::jackal_msgs::msg::DriveFeedback & msg)
  : msg_(msg)
  {}
  Init_DriveFeedback_bridge_temperature duty_cycle(::jackal_msgs::msg::DriveFeedback::_duty_cycle_type arg)
  {
    msg_.duty_cycle = std::move(arg);
    return Init_DriveFeedback_bridge_temperature(msg_);
  }

private:
  ::jackal_msgs::msg::DriveFeedback msg_;
};

class Init_DriveFeedback_current
{
public:
  Init_DriveFeedback_current()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveFeedback_duty_cycle current(::jackal_msgs::msg::DriveFeedback::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_DriveFeedback_duty_cycle(msg_);
  }

private:
  ::jackal_msgs::msg::DriveFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jackal_msgs::msg::DriveFeedback>()
{
  return jackal_msgs::msg::builder::Init_DriveFeedback_current();
}

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__BUILDER_HPP_
