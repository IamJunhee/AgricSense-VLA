// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jackal_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__FEEDBACK__TRAITS_HPP_
#define JACKAL_MSGS__MSG__DETAIL__FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jackal_msgs/msg/detail/feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'drivers'
#include "jackal_msgs/msg/detail/drive_feedback__traits.hpp"

namespace jackal_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
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
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: commanded_mode
  {
    out << "commanded_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.commanded_mode, out);
    out << ", ";
  }

  // member: actual_mode
  {
    out << "actual_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Feedback & msg,
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
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: commanded_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "commanded_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.commanded_mode, out);
    out << "\n";
  }

  // member: actual_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "actual_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.actual_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Feedback & msg, bool use_flow_style = false)
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
  const jackal_msgs::msg::Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  jackal_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jackal_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const jackal_msgs::msg::Feedback & msg)
{
  return jackal_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jackal_msgs::msg::Feedback>()
{
  return "jackal_msgs::msg::Feedback";
}

template<>
inline const char * name<jackal_msgs::msg::Feedback>()
{
  return "jackal_msgs/msg/Feedback";
}

template<>
struct has_fixed_size<jackal_msgs::msg::Feedback>
  : std::integral_constant<bool, has_fixed_size<jackal_msgs::msg::DriveFeedback>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<jackal_msgs::msg::Feedback>
  : std::integral_constant<bool, has_bounded_size<jackal_msgs::msg::DriveFeedback>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<jackal_msgs::msg::Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JACKAL_MSGS__MSG__DETAIL__FEEDBACK__TRAITS_HPP_
