// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosbasic_move_msgs:srv/Control.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__STRUCT_HPP_
#define ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosbasic_move_msgs__srv__Control_Request __attribute__((deprecated))
#else
# define DEPRECATED__rosbasic_move_msgs__srv__Control_Request __declspec(deprecated)
#endif

namespace rosbasic_move_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Control_Request_
{
  using Type = Control_Request_<ContainerAllocator>;

  explicit Control_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quangduong = 0.0f;
      this->khoangcach = 0.0f;
    }
  }

  explicit Control_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->quangduong = 0.0f;
      this->khoangcach = 0.0f;
    }
  }

  // field types and members
  using _quangduong_type =
    float;
  _quangduong_type quangduong;
  using _khoangcach_type =
    float;
  _khoangcach_type khoangcach;

  // setters for named parameter idiom
  Type & set__quangduong(
    const float & _arg)
  {
    this->quangduong = _arg;
    return *this;
  }
  Type & set__khoangcach(
    const float & _arg)
  {
    this->khoangcach = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbasic_move_msgs__srv__Control_Request
    std::shared_ptr<rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbasic_move_msgs__srv__Control_Request
    std::shared_ptr<rosbasic_move_msgs::srv::Control_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Control_Request_ & other) const
  {
    if (this->quangduong != other.quangduong) {
      return false;
    }
    if (this->khoangcach != other.khoangcach) {
      return false;
    }
    return true;
  }
  bool operator!=(const Control_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Control_Request_

// alias to use template instance with default allocator
using Control_Request =
  rosbasic_move_msgs::srv::Control_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbasic_move_msgs


#ifndef _WIN32
# define DEPRECATED__rosbasic_move_msgs__srv__Control_Response __attribute__((deprecated))
#else
# define DEPRECATED__rosbasic_move_msgs__srv__Control_Response __declspec(deprecated)
#endif

namespace rosbasic_move_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Control_Response_
{
  using Type = Control_Response_<ContainerAllocator>;

  explicit Control_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->notification = "";
    }
  }

  explicit Control_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : notification(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->notification = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _notification_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _notification_type notification;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__notification(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->notification = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbasic_move_msgs__srv__Control_Response
    std::shared_ptr<rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbasic_move_msgs__srv__Control_Response
    std::shared_ptr<rosbasic_move_msgs::srv::Control_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Control_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->notification != other.notification) {
      return false;
    }
    return true;
  }
  bool operator!=(const Control_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Control_Response_

// alias to use template instance with default allocator
using Control_Response =
  rosbasic_move_msgs::srv::Control_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbasic_move_msgs

namespace rosbasic_move_msgs
{

namespace srv
{

struct Control
{
  using Request = rosbasic_move_msgs::srv::Control_Request;
  using Response = rosbasic_move_msgs::srv::Control_Response;
};

}  // namespace srv

}  // namespace rosbasic_move_msgs

#endif  // ROSBASIC_MOVE_MSGS__SRV__DETAIL__CONTROL__STRUCT_HPP_
