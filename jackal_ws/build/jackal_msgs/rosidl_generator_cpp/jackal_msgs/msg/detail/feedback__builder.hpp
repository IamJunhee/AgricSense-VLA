// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jackal_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
#define JACKAL_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jackal_msgs/msg/detail/feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jackal_msgs
{

namespace msg
{

namespace builder
{

class Init_Feedback_actual_mode
{
public:
  explicit Init_Feedback_actual_mode(::jackal_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  ::jackal_msgs::msg::Feedback actual_mode(::jackal_msgs::msg::Feedback::_actual_mode_type arg)
  {
    msg_.actual_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jackal_msgs::msg::Feedback msg_;
};

class Init_Feedback_commanded_mode
{
public:
  explicit Init_Feedback_commanded_mode(::jackal_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_actual_mode commanded_mode(::jackal_msgs::msg::Feedback::_commanded_mode_type arg)
  {
    msg_.commanded_mode = std::move(arg);
    return Init_Feedback_actual_mode(msg_);
  }

private:
  ::jackal_msgs::msg::Feedback msg_;
};

class Init_Feedback_drivers
{
public:
  explicit Init_Feedback_drivers(::jackal_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_commanded_mode drivers(::jackal_msgs::msg::Feedback::_drivers_type arg)
  {
    msg_.drivers = std::move(arg);
    return Init_Feedback_commanded_mode(msg_);
  }

private:
  ::jackal_msgs::msg::Feedback msg_;
};

class Init_Feedback_header
{
public:
  Init_Feedback_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Feedback_drivers header(::jackal_msgs::msg::Feedback::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Feedback_drivers(msg_);
  }

private:
  ::jackal_msgs::msg::Feedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jackal_msgs::msg::Feedback>()
{
  return jackal_msgs::msg::builder::Init_Feedback_header();
}

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
