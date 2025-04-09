// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jackal_msgs:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__DRIVE__TRAITS_HPP_
#define JACKAL_MSGS__MSG__DETAIL__DRIVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jackal_msgs/msg/detail/drive__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace jackal_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Drive & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: drivers
  {
    if (msg.drivers.size() == 0) {
      out << "drivers: []";
    } else {
      out << "drivers: [";
      size_t pending_items = msg.drivers.size();
      for (auto item : msg.drivers) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Drive & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: drivers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.drivers.size() == 0) {
      out << "drivers: []\n";
    } else {
      out << "drivers:\n";
      for (auto item : msg.drivers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Drive & msg, bool use_flow_style = false)
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
  const jackal_msgs::msg::Drive & msg,
  std::ostream & out, size_t indentation = 0)
{
  jackal_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jackal_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const jackal_msgs::msg::Drive & msg)
{
  return jackal_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jackal_msgs::msg::Drive>()
{
  return "jackal_msgs::msg::Drive";
}

template<>
inline const char * name<jackal_msgs::msg::Drive>()
{
  return "jackal_msgs/msg/Drive";
}

template<>
struct has_fixed_size<jackal_msgs::msg::Drive>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<jackal_msgs::msg::Drive>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<jackal_msgs::msg::Drive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JACKAL_MSGS__MSG__DETAIL__DRIVE__TRAITS_HPP_
