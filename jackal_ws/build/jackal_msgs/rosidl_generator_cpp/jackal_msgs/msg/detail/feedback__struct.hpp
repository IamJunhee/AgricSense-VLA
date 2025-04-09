// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jackal_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef JACKAL_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_HPP_
#define JACKAL_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_HPP_

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
// Member 'drivers'
#include "jackal_msgs/msg/detail/drive_feedback__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jackal_msgs__msg__Feedback __attribute__((deprecated))
#else
# define DEPRECATED__jackal_msgs__msg__Feedback __declspec(deprecated)
#endif

namespace jackal_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Feedback_
{
  using Type = Feedback_<ContainerAllocator>;

  explicit Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drivers.fill(jackal_msgs::msg::DriveFeedback_<ContainerAllocator>{_init});
      this->commanded_mode = 0;
      this->actual_mode = 0;
    }
  }

  explicit Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    drivers(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drivers.fill(jackal_msgs::msg::DriveFeedback_<ContainerAllocator>{_alloc, _init});
      this->commanded_mode = 0;
      this->actual_mode = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _drivers_type =
    std::array<jackal_msgs::msg::DriveFeedback_<ContainerAllocator>, 2>;
  _drivers_type drivers;
  using _commanded_mode_type =
    int8_t;
  _commanded_mode_type commanded_mode;
  using _actual_mode_type =
    int8_t;
  _actual_mode_type actual_mode;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__drivers(
    const std::array<jackal_msgs::msg::DriveFeedback_<ContainerAllocator>, 2> & _arg)
  {
    this->drivers = _arg;
    return *this;
  }
  Type & set__commanded_mode(
    const int8_t & _arg)
  {
    this->commanded_mode = _arg;
    return *this;
  }
  Type & set__actual_mode(
    const int8_t & _arg)
  {
    this->actual_mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jackal_msgs::msg::Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const jackal_msgs::msg::Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jackal_msgs::msg::Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jackal_msgs::msg::Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jackal_msgs::msg::Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jackal_msgs::msg::Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jackal_msgs::msg::Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jackal_msgs::msg::Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jackal_msgs::msg::Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jackal_msgs::msg::Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jackal_msgs__msg__Feedback
    std::shared_ptr<jackal_msgs::msg::Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jackal_msgs__msg__Feedback
    std::shared_ptr<jackal_msgs::msg::Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Feedback_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->drivers != other.drivers) {
      return false;
    }
    if (this->commanded_mode != other.commanded_mode) {
      return false;
    }
    if (this->actual_mode != other.actual_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Feedback_

// alias to use template instance with default allocator
using Feedback =
  jackal_msgs::msg::Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace jackal_msgs

#endif  // JACKAL_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_HPP_
