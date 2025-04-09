// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jackal_msgs:srv/SetDomainId.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__SRV__DETAIL__SET_DOMAIN_ID__BUILDER_HPP_
#define JACKAL_MSGS__SRV__DETAIL__SET_DOMAIN_ID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jackal_msgs/srv/detail/set_domain_id__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jackal_msgs
{

namespace srv
{

namespace builder
{

class Init_SetDomainId_Request_domain_id
{
public:
  Init_SetDomainId_Request_domain_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jackal_msgs::srv::SetDomainId_Request domain_id(::jackal_msgs::srv::SetDomainId_Request::_domain_id_type arg)
  {
    msg_.domain_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jackal_msgs::srv::SetDomainId_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jackal_msgs::srv::SetDomainId_Request>()
{
  return jackal_msgs::srv::builder::Init_SetDomainId_Request_domain_id();
}

}  // namespace jackal_msgs


namespace jackal_msgs
{

namespace srv
{

namespace builder
{

class Init_SetDomainId_Response_message
{
public:
  explicit Init_SetDomainId_Response_message(::jackal_msgs::srv::SetDomainId_Response & msg)
  : msg_(msg)
  {}
  ::jackal_msgs::srv::SetDomainId_Response message(::jackal_msgs::srv::SetDomainId_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jackal_msgs::srv::SetDomainId_Response msg_;
};

class Init_SetDomainId_Response_success
{
public:
  Init_SetDomainId_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetDomainId_Response_message success(::jackal_msgs::srv::SetDomainId_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetDomainId_Response_message(msg_);
  }

private:
  ::jackal_msgs::srv::SetDomainId_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jackal_msgs::srv::SetDomainId_Response>()
{
  return jackal_msgs::srv::builder::Init_SetDomainId_Response_success();
}

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__SRV__DETAIL__SET_DOMAIN_ID__BUILDER_HPP_
