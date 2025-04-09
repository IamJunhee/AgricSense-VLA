// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jackal_msgs:msg/StopStatus.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__STOP_STATUS__TRAITS_HPP_
#define JACKAL_MSGS__MSG__DETAIL__STOP_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jackal_msgs/msg/detail/stop_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace jackal_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const StopStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: stop_power_status
  {
    out << "stop_power_status: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_power_status, out);
    out << ", ";
  }

  // member: external_stop_present
  {
    out << "external_stop_present: ";
    rosidl_generator_traits::value_to_yaml(msg.external_stop_present, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StopStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: stop_power_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop_power_status: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_power_status, out);
    out << "\n";
  }

  // member: external_stop_present
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "external_stop_present: ";
    rosidl_generator_traits::value_to_yaml(msg.external_stop_present, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StopStatus & msg, bool use_flow_style = false)
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

}  // namespace jackal_msgs

namespace rosidl_generator_traits
{

[[deprecated("use jackal_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jackal_msgs::msg::StopStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  jackal_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jackal_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const jackal_msgs::msg::StopStatus & msg)
{
  return jackal_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jackal_msgs::msg::StopStatus>()
{
  return "jackal_msgs::msg::StopStatus";
}

template<>
inline const char * name<jackal_msgs::msg::StopStatus>()
{
  return "jackal_msgs/msg/StopStatus";
}

template<>
struct has_fixed_size<jackal_msgs::msg::StopStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<jackal_msgs::msg::StopStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<jackal_msgs::msg::StopStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JACKAL_MSGS__MSG__DETAIL__STOP_STATUS__TRAITS_HPP_
