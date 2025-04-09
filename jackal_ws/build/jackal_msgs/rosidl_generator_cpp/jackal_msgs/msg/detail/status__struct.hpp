// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jackal_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
#define JACKAL_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'mcu_uptime'
// Member 'connection_uptime'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jackal_msgs__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__jackal_msgs__msg__Status __declspec(deprecated)
#endif

namespace jackal_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    mcu_uptime(_init),
    connection_uptime(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hardware_id = "";
      this->firmware_commit = "";
      this->pcb_temperature = 0.0f;
      this->mcu_temperature = 0.0f;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    hardware_id(_alloc),
    firmware_commit(_alloc),
    mcu_uptime(_alloc, _init),
    connection_uptime(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hardware_id = "";
      this->firmware_commit = "";
      this->pcb_temperature = 0.0f;
      this->mcu_temperature = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _hardware_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _hardware_id_type hardware_id;
  using _firmware_commit_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _firmware_commit_type firmware_commit;
  using _mcu_uptime_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _mcu_uptime_type mcu_uptime;
  using _connection_uptime_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _connection_uptime_type connection_uptime;
  using _pcb_temperature_type =
    float;
  _pcb_temperature_type pcb_temperature;
  using _mcu_temperature_type =
    float;
  _mcu_temperature_type mcu_temperature;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__hardware_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->hardware_id = _arg;
    return *this;
  }
  Type & set__firmware_commit(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->firmware_commit = _arg;
    return *this;
  }
  Type & set__mcu_uptime(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->mcu_uptime = _arg;
    return *this;
  }
  Type & set__connection_uptime(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->connection_uptime = _arg;
    return *this;
  }
  Type & set__pcb_temperature(
    const float & _arg)
  {
    this->pcb_temperature = _arg;
    return *this;
  }
  Type & set__mcu_temperature(
    const float & _arg)
  {
    this->mcu_temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jackal_msgs::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const jackal_msgs::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jackal_msgs::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jackal_msgs::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jackal_msgs::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jackal_msgs::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jackal_msgs::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jackal_msgs::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jackal_msgs::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jackal_msgs::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jackal_msgs__msg__Status
    std::shared_ptr<jackal_msgs::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jackal_msgs__msg__Status
    std::shared_ptr<jackal_msgs::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->hardware_id != other.hardware_id) {
      return false;
    }
    if (this->firmware_commit != other.firmware_commit) {
      return false;
    }
    if (this->mcu_uptime != other.mcu_uptime) {
      return false;
    }
    if (this->connection_uptime != other.connection_uptime) {
      return false;
    }
    if (this->pcb_temperature != other.pcb_temperature) {
      return false;
    }
    if (this->mcu_temperature != other.mcu_temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  jackal_msgs::msg::Status_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
