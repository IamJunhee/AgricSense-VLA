// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jackal_msgs:msg/DriveFeedback.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__TRAITS_HPP_
#define JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jackal_msgs/msg/detail/drive_feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace jackal_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DriveFeedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << ", ";
  }

  // member: duty_cycle
  {
    out << "duty_cycle: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle, out);
    out << ", ";
  }

  // member: bridge_temperature
  {
    out << "bridge_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.bridge_temperature, out);
    out << ", ";
  }

  // member: motor_temperature
  {
    out << "motor_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_temperature, out);
    out << ", ";
  }

  // member: measured_velocity
  {
    out << "measured_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_velocity, out);
    out << ", ";
  }

  // member: measured_travel
  {
    out << "measured_travel: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_travel, out);
    out << ", ";
  }

  // member: driver_fault
  {
    out << "driver_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_fault, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DriveFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << "\n";
  }

  // member: duty_cycle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duty_cycle: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle, out);
    out << "\n";
  }

  // member: bridge_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bridge_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.bridge_temperature, out);
    out << "\n";
  }

  // member: motor_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_temperature, out);
    out << "\n";
  }

  // member: measured_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_velocity, out);
    out << "\n";
  }

  // member: measured_travel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "measured_travel: ";
    rosidl_generator_traits::value_to_yaml(msg.measured_travel, out);
    out << "\n";
  }

  // member: driver_fault
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "driver_fault: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_fault, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DriveFeedback & msg, bool use_flow_style = false)
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
  const jackal_msgs::msg::DriveFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  jackal_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jackal_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const jackal_msgs::msg::DriveFeedback & msg)
{
  return jackal_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jackal_msgs::msg::DriveFeedback>()
{
  return "jackal_msgs::msg::DriveFeedback";
}

template<>
inline const char * name<jackal_msgs::msg::DriveFeedback>()
{
  return "jackal_msgs/msg/DriveFeedback";
}

template<>
struct has_fixed_size<jackal_msgs::msg::DriveFeedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<jackal_msgs::msg::DriveFeedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<jackal_msgs::msg::DriveFeedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__TRAITS_HPP_
