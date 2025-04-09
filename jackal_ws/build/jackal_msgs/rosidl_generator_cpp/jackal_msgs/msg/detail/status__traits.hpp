// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jackal_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
#define JACKAL_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jackal_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'mcu_uptime'
// Member 'connection_uptime'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace jackal_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: hardware_id
  {
    out << "hardware_id: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_id, out);
    out << ", ";
  }

  // member: firmware_commit
  {
    out << "firmware_commit: ";
    rosidl_generator_traits::value_to_yaml(msg.firmware_commit, out);
    out << ", ";
  }

  // member: mcu_uptime
  {
    out << "mcu_uptime: ";
    to_flow_style_yaml(msg.mcu_uptime, out);
    out << ", ";
  }

  // member: connection_uptime
  {
    out << "connection_uptime: ";
    to_flow_style_yaml(msg.connection_uptime, out);
    out << ", ";
  }

  // member: pcb_temperature
  {
    out << "pcb_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.pcb_temperature, out);
    out << ", ";
  }

  // member: mcu_temperature
  {
    out << "mcu_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.mcu_temperature, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Status & msg,
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

  // member: hardware_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hardware_id: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_id, out);
    out << "\n";
  }

  // member: firmware_commit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "firmware_commit: ";
    rosidl_generator_traits::value_to_yaml(msg.firmware_commit, out);
    out << "\n";
  }

  // member: mcu_uptime
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mcu_uptime:\n";
    to_block_style_yaml(msg.mcu_uptime, out, indentation + 2);
  }

  // member: connection_uptime
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "connection_uptime:\n";
    to_block_style_yaml(msg.connection_uptime, out, indentation + 2);
  }

  // member: pcb_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pcb_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.pcb_temperature, out);
    out << "\n";
  }

  // member: mcu_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mcu_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.mcu_temperature, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Status & msg, bool use_flow_style = false)
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
  const jackal_msgs::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  jackal_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jackal_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const jackal_msgs::msg::Status & msg)
{
  return jackal_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jackal_msgs::msg::Status>()
{
  return "jackal_msgs::msg::Status";
}

template<>
inline const char * name<jackal_msgs::msg::Status>()
{
  return "jackal_msgs/msg/Status";
}

template<>
struct has_fixed_size<jackal_msgs::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jackal_msgs::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jackal_msgs::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JACKAL_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
