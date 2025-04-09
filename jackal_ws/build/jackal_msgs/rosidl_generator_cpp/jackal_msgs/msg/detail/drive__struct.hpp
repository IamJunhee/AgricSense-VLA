// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jackal_msgs:msg/Drive.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__DRIVE__STRUCT_HPP_
#define JACKAL_MSGS__MSG__DETAIL__DRIVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__jackal_msgs__msg__Drive __attribute__((deprecated))
#else
# define DEPRECATED__jackal_msgs__msg__Drive __declspec(deprecated)
#endif

namespace jackal_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Drive_
{
  using Type = Drive_<ContainerAllocator>;

  explicit Drive_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      std::fill<typename std::array<float, 2>::iterator, float>(this->drivers.begin(), this->drivers.end(), 0.0f);
    }
  }

  explicit Drive_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : drivers(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      std::fill<typename std::array<float, 2>::iterator, float>(this->drivers.begin(), this->drivers.end(), 0.0f);
    }
  }

  // field types and members
  using _mode_type =
    int8_t;
  _mode_type mode;
  using _drivers_type =
    std::array<float, 2>;
  _drivers_type drivers;

  // setters for named parameter idiom
  Type & set__mode(
    const int8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__drivers(
    const std::array<float, 2> & _arg)
  {
    this->drivers = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t MODE_VELOCITY =
    0;
  static constexpr int8_t MODE_PWM =
    1;
  static constexpr int8_t MODE_EFFORT =
    2;
  static constexpr int8_t MODE_NONE =
    -1;
  static constexpr int8_t LEFT =
    0;
  static constexpr int8_t RIGHT =
    1;

  // pointer types
  using RawPtr =
    jackal_msgs::msg::Drive_<ContainerAllocator> *;
  using ConstRawPtr =
    const jackal_msgs::msg::Drive_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jackal_msgs::msg::Drive_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jackal_msgs::msg::Drive_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jackal_msgs::msg::Drive_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jackal_msgs::msg::Drive_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jackal_msgs::msg::Drive_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jackal_msgs::msg::Drive_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jackal_msgs::msg::Drive_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jackal_msgs::msg::Drive_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jackal_msgs__msg__Drive
    std::shared_ptr<jackal_msgs::msg::Drive_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jackal_msgs__msg__Drive
    std::shared_ptr<jackal_msgs::msg::Drive_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->drivers != other.drivers) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_

// alias to use template instance with default allocator
using Drive =
  jackal_msgs::msg::Drive_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t Drive_<ContainerAllocator>::MODE_VELOCITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t Drive_<ContainerAllocator>::MODE_PWM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t Drive_<ContainerAllocator>::MODE_EFFORT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t Drive_<ContainerAllocator>::MODE_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t Drive_<ContainerAllocator>::LEFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t Drive_<ContainerAllocator>::RIGHT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__MSG__DETAIL__DRIVE__STRUCT_HPP_
