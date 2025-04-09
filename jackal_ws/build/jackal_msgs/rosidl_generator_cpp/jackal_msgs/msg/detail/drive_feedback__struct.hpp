// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jackal_msgs:msg/DriveFeedback.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__STRUCT_HPP_
#define JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__jackal_msgs__msg__DriveFeedback __attribute__((deprecated))
#else
# define DEPRECATED__jackal_msgs__msg__DriveFeedback __declspec(deprecated)
#endif

namespace jackal_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DriveFeedback_
{
  using Type = DriveFeedback_<ContainerAllocator>;

  explicit DriveFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current = 0.0f;
      this->duty_cycle = 0.0f;
      this->bridge_temperature = 0.0f;
      this->motor_temperature = 0.0f;
      this->measured_velocity = 0.0f;
      this->measured_travel = 0.0f;
      this->driver_fault = false;
    }
  }

  explicit DriveFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current = 0.0f;
      this->duty_cycle = 0.0f;
      this->bridge_temperature = 0.0f;
      this->motor_temperature = 0.0f;
      this->measured_velocity = 0.0f;
      this->measured_travel = 0.0f;
      this->driver_fault = false;
    }
  }

  // field types and members
  using _current_type =
    float;
  _current_type current;
  using _duty_cycle_type =
    float;
  _duty_cycle_type duty_cycle;
  using _bridge_temperature_type =
    float;
  _bridge_temperature_type bridge_temperature;
  using _motor_temperature_type =
    float;
  _motor_temperature_type motor_temperature;
  using _measured_velocity_type =
    float;
  _measured_velocity_type measured_velocity;
  using _measured_travel_type =
    float;
  _measured_travel_type measured_travel;
  using _driver_fault_type =
    bool;
  _driver_fault_type driver_fault;

  // setters for named parameter idiom
  Type & set__current(
    const float & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__duty_cycle(
    const float & _arg)
  {
    this->duty_cycle = _arg;
    return *this;
  }
  Type & set__bridge_temperature(
    const float & _arg)
  {
    this->bridge_temperature = _arg;
    return *this;
  }
  Type & set__motor_temperature(
    const float & _arg)
  {
    this->motor_temperature = _arg;
    return *this;
  }
  Type & set__measured_velocity(
    const float & _arg)
  {
    this->measured_velocity = _arg;
    return *this;
  }
  Type & set__measured_travel(
    const float & _arg)
  {
    this->measured_travel = _arg;
    return *this;
  }
  Type & set__driver_fault(
    const bool & _arg)
  {
    this->driver_fault = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jackal_msgs::msg::DriveFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const jackal_msgs::msg::DriveFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jackal_msgs::msg::DriveFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jackal_msgs::msg::DriveFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jackal_msgs::msg::DriveFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jackal_msgs::msg::DriveFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jackal_msgs::msg::DriveFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jackal_msgs::msg::DriveFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jackal_msgs::msg::DriveFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jackal_msgs::msg::DriveFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jackal_msgs__msg__DriveFeedback
    std::shared_ptr<jackal_msgs::msg::DriveFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jackal_msgs__msg__DriveFeedback
    std::shared_ptr<jackal_msgs::msg::DriveFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveFeedback_ & other) const
  {
    if (this->current != other.current) {
      return false;
    }
    if (this->duty_cycle != other.duty_cycle) {
      return false;
    }
    if (this->bridge_temperature != other.bridge_temperature) {
      return false;
    }
    if (this->motor_temperature != other.motor_temperature) {
      return false;
    }
    if (this->measured_velocity != other.measured_velocity) {
      return false;
    }
    if (this->measured_travel != other.measured_travel) {
      return false;
    }
    if (this->driver_fault != other.driver_fault) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveFeedback_

// alias to use template instance with default allocator
using DriveFeedback =
  jackal_msgs::msg::DriveFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__MSG__DETAIL__DRIVE_FEEDBACK__STRUCT_HPP_
