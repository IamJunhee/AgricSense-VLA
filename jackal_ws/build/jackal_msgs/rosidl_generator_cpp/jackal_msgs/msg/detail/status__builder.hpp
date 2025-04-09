// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jackal_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
#define JACKAL_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jackal_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jackal_msgs
{

namespace msg
{

namespace builder
{

class Init_Status_mcu_temperature
{
public:
  explicit Init_Status_mcu_temperature(::jackal_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  ::jackal_msgs::msg::Status mcu_temperature(::jackal_msgs::msg::Status::_mcu_temperature_type arg)
  {
    msg_.mcu_temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jackal_msgs::msg::Status msg_;
};

class Init_Status_pcb_temperature
{
public:
  explicit Init_Status_pcb_temperature(::jackal_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_mcu_temperature pcb_temperature(::jackal_msgs::msg::Status::_pcb_temperature_type arg)
  {
    msg_.pcb_temperature = std::move(arg);
    return Init_Status_mcu_temperature(msg_);
  }

private:
  ::jackal_msgs::msg::Status msg_;
};

class Init_Status_connection_uptime
{
public:
  explicit Init_Status_connection_uptime(::jackal_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_pcb_temperature connection_uptime(::jackal_msgs::msg::Status::_connection_uptime_type arg)
  {
    msg_.connection_uptime = std::move(arg);
    return Init_Status_pcb_temperature(msg_);
  }

private:
  ::jackal_msgs::msg::Status msg_;
};

class Init_Status_mcu_uptime
{
public:
  explicit Init_Status_mcu_uptime(::jackal_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_connection_uptime mcu_uptime(::jackal_msgs::msg::Status::_mcu_uptime_type arg)
  {
    msg_.mcu_uptime = std::move(arg);
    return Init_Status_connection_uptime(msg_);
  }

private:
  ::jackal_msgs::msg::Status msg_;
};

class Init_Status_firmware_commit
{
public:
  explicit Init_Status_firmware_commit(::jackal_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_mcu_uptime firmware_commit(::jackal_msgs::msg::Status::_firmware_commit_type arg)
  {
    msg_.firmware_commit = std::move(arg);
    return Init_Status_mcu_uptime(msg_);
  }

private:
  ::jackal_msgs::msg::Status msg_;
};

class Init_Status_hardware_id
{
public:
  explicit Init_Status_hardware_id(::jackal_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_firmware_commit hardware_id(::jackal_msgs::msg::Status::_hardware_id_type arg)
  {
    msg_.hardware_id = std::move(arg);
    return Init_Status_firmware_commit(msg_);
  }

private:
  ::jackal_msgs::msg::Status msg_;
};

class Init_Status_header
{
public:
  Init_Status_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_hardware_id header(::jackal_msgs::msg::Status::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Status_hardware_id(msg_);
  }

private:
  ::jackal_msgs::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jackal_msgs::msg::Status>()
{
  return jackal_msgs::msg::builder::Init_Status_header();
}

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
