// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from jackal_msgs:msg/Drive.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "jackal_msgs/msg/detail/drive__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace jackal_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Drive_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) jackal_msgs::msg::Drive(_init);
}

void Drive_fini_function(void * message_memory)
{
  auto typed_message = static_cast<jackal_msgs::msg::Drive *>(message_memory);
  typed_message->~Drive();
}

size_t size_function__Drive__drivers(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__Drive__drivers(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__Drive__drivers(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__Drive__drivers(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__Drive__drivers(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__Drive__drivers(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__Drive__drivers(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Drive_message_member_array[2] = {
  {
    "mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs::msg::Drive, mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "drivers",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(jackal_msgs::msg::Drive, drivers),  // bytes offset in struct
    nullptr,  // default value
    size_function__Drive__drivers,  // size() function pointer
    get_const_function__Drive__drivers,  // get_const(index) function pointer
    get_function__Drive__drivers,  // get(index) function pointer
    fetch_function__Drive__drivers,  // fetch(index, &value) function pointer
    assign_function__Drive__drivers,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Drive_message_members = {
  "jackal_msgs::msg",  // message namespace
  "Drive",  // message name
  2,  // number of fields
  sizeof(jackal_msgs::msg::Drive),
  Drive_message_member_array,  // message members
  Drive_init_function,  // function to initialize message memory (memory has to be allocated)
  Drive_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Drive_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Drive_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace jackal_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jackal_msgs::msg::Drive>()
{
  return &::jackal_msgs::msg::rosidl_typesupport_introspection_cpp::Drive_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jackal_msgs, msg, Drive)() {
  return &::jackal_msgs::msg::rosidl_typesupport_introspection_cpp::Drive_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
