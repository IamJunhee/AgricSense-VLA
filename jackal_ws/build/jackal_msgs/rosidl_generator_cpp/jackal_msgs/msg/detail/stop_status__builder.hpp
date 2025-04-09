// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jackal_msgs:msg/StopStatus.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__STOP_STATUS__BUILDER_HPP_
#define JACKAL_MSGS__MSG__DETAIL__STOP_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jackal_msgs/msg/detail/stop_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jackal_msgs
{

namespace msg
{

namespace builder
{

class Init_StopStatus_external_stop_present
{
public:
  explicit Init_StopStatus_external_stop_present(::jackal_msgs::msg::StopStatus & msg)
  : msg_(msg)
  {}
  ::jackal_msgs::msg::StopStatus external_stop_present(::jackal_msgs::msg::StopStatus::_external_stop_present_type arg)
  {
    msg_.external_stop_present = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jackal_msgs::msg::StopStatus msg_;
};

class Init_StopStatus_stop_power_status
{
public:
  explicit Init_StopStatus_stop_power_status(::jackal_msgs::msg::StopStatus & msg)
  : msg_(msg)
  {}
  Init_StopStatus_external_stop_present stop_power_status(::jackal_msgs::msg::StopStatus::_stop_power_status_type arg)
  {
    msg_.stop_power_status = std::move(arg);
    return Init_StopStatus_external_stop_present(msg_);
  }

private:
  ::jackal_msgs::msg::StopStatus msg_;
};

class Init_StopStatus_header
{
public:
  Init_StopStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StopStatus_stop_power_status header(::jackal_msgs::msg::StopStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StopStatus_stop_power_status(msg_);
  }

private:
  ::jackal_msgs::msg::StopStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jackal_msgs::msg::StopStatus>()
{
  return jackal_msgs::msg::builder::Init_StopStatus_header();
}

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__MSG__DETAIL__STOP_STATUS__BUILDER_HPP_
