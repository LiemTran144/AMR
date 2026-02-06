// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosbasic_msgs:srv/Multi.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MSGS__SRV__DETAIL__MULTI__STRUCT_HPP_
#define ROSBASIC_MSGS__SRV__DETAIL__MULTI__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosbasic_msgs__srv__Multi_Request __attribute__((deprecated))
#else
# define DEPRECATED__rosbasic_msgs__srv__Multi_Request __declspec(deprecated)
#endif

namespace rosbasic_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Multi_Request_
{
  using Type = Multi_Request_<ContainerAllocator>;

  explicit Multi_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0ll;
      this->b = 0ll;
    }
  }

  explicit Multi_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->a = 0ll;
      this->b = 0ll;
    }
  }

  // field types and members
  using _a_type =
    int64_t;
  _a_type a;
  using _b_type =
    int64_t;
  _b_type b;

  // setters for named parameter idiom
  Type & set__a(
    const int64_t & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__b(
    const int64_t & _arg)
  {
    this->b = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosbasic_msgs::srv::Multi_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbasic_msgs::srv::Multi_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbasic_msgs::srv::Multi_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbasic_msgs::srv::Multi_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbasic_msgs::srv::Multi_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbasic_msgs::srv::Multi_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbasic_msgs::srv::Multi_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbasic_msgs::srv::Multi_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbasic_msgs::srv::Multi_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbasic_msgs::srv::Multi_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbasic_msgs__srv__Multi_Request
    std::shared_ptr<rosbasic_msgs::srv::Multi_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbasic_msgs__srv__Multi_Request
    std::shared_ptr<rosbasic_msgs::srv::Multi_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Multi_Request_ & other) const
  {
    if (this->a != other.a) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    return true;
  }
  bool operator!=(const Multi_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Multi_Request_

// alias to use template instance with default allocator
using Multi_Request =
  rosbasic_msgs::srv::Multi_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbasic_msgs


#ifndef _WIN32
# define DEPRECATED__rosbasic_msgs__srv__Multi_Response __attribute__((deprecated))
#else
# define DEPRECATED__rosbasic_msgs__srv__Multi_Response __declspec(deprecated)
#endif

namespace rosbasic_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Multi_Response_
{
  using Type = Multi_Response_<ContainerAllocator>;

  explicit Multi_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mul = 0ll;
    }
  }

  explicit Multi_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mul = 0ll;
    }
  }

  // field types and members
  using _mul_type =
    int64_t;
  _mul_type mul;

  // setters for named parameter idiom
  Type & set__mul(
    const int64_t & _arg)
  {
    this->mul = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosbasic_msgs::srv::Multi_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbasic_msgs::srv::Multi_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbasic_msgs::srv::Multi_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbasic_msgs::srv::Multi_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbasic_msgs::srv::Multi_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbasic_msgs::srv::Multi_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbasic_msgs::srv::Multi_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbasic_msgs::srv::Multi_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbasic_msgs::srv::Multi_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbasic_msgs::srv::Multi_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbasic_msgs__srv__Multi_Response
    std::shared_ptr<rosbasic_msgs::srv::Multi_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbasic_msgs__srv__Multi_Response
    std::shared_ptr<rosbasic_msgs::srv::Multi_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Multi_Response_ & other) const
  {
    if (this->mul != other.mul) {
      return false;
    }
    return true;
  }
  bool operator!=(const Multi_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Multi_Response_

// alias to use template instance with default allocator
using Multi_Response =
  rosbasic_msgs::srv::Multi_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbasic_msgs

namespace rosbasic_msgs
{

namespace srv
{

struct Multi
{
  using Request = rosbasic_msgs::srv::Multi_Request;
  using Response = rosbasic_msgs::srv::Multi_Response;
};

}  // namespace srv

}  // namespace rosbasic_msgs

#endif  // ROSBASIC_MSGS__SRV__DETAIL__MULTI__STRUCT_HPP_
