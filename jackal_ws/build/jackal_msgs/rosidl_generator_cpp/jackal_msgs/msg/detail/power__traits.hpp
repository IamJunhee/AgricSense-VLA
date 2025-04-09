// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jackal_msgs:msg/Power.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__POWER__TRAITS_HPP_
#define JACKAL_MSGS__MSG__DETAIL__POWER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jackal_msgs/msg/detail/power__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace jackal_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Power & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: shore_power_connected
  {
    out << "shore_power_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.shore_power_connected, out);
    out << ", ";
  }

  // member: battery_connected
  {
    out << "battery_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_connected, out);
    out << ", ";
  }

  // member: power_12v_user_nominal
  {
    out << "power_12v_user_nominal: ";
    rosidl_generator_traits::value_to_yaml(msg.power_12v_user_nominal, out);
    out << ", ";
  }

  // member: charging_complete
  {
    out << "charging_complete: ";
    rosidl_generator_traits::value_to_yaml(msg.charging_complete, out);
    out << ", ";
  }

  // member: measured_voltages
  {
    if (msg.measured_voltages.size() == 0) {
      out << "measured_voltages: []";
    } else {
      out << "measured_voltages: [";
      size_t pending_items = msg.measured_voltages.size();
      for (auto item : msg.measured_voltages) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: measured_currents
  {
    if (msg.measured_currents.size() == 0) {
      out << "measured_currents: []";
    } else {
      out << "measured_currents: [";
      size_t pending_items = msg.measured_currents.size();
      for (auto item : msg.measured_currents) {
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
  const Power & msg,
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

  // member: shore_power_connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shore_power_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.shore_power_connected, out);
    out << "\n";
  }

  // member: battery_connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_connected: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_connected, out);
    out << "\n";
  }

  // member: power_12v_user_nominal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "power_12v_user_nominal: ";
    rosidl_generator_traits::value_to_yaml(msg.power_12v_user_nominal, out);
    out << "\n";
  }

  // member: charging_complete
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charging_complete: ";
    rosidl_generator_traits::value_to_yaml(msg.charging_complete, out);
    out << "\n";
  }

  // member: measured_voltages
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.measured_voltages.size() == 0) {
      out << "measured_voltages: []\n";
    } else {
      out << "measured_voltages:\n";
      for (auto item : msg.measured_voltages) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: measured_currents
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.measured_currents.size() == 0) {
      out << "measured_currents: []\n";
    } else {
      out << "measured_currents:\n";
      for (auto item : msg.measured_currents) {
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

inline std::string to_yaml(const Power & msg, bool use_flow_style = false)
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
  const jackal_msgs::msg::Power & msg,
  std::ostream & out, size_t indentation = 0)
{
  jackal_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jackal_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const jackal_msgs::msg::Power & msg)
{
  return jackal_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jackal_msgs::msg::Power>()
{
  return "jackal_msgs::msg::Power";
}

template<>
inline const char * name<jackal_msgs::msg::Power>()
{
  return "jackal_msgs/msg/Power";
}

template<>
struct has_fixed_size<jackal_msgs::msg::Power>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jackal_msgs::msg::Power>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jackal_msgs::msg::Power>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JACKAL_MSGS__MSG__DETAIL__POWER__TRAITS_HPP_
