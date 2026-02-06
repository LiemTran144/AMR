// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosbasic_msgs:srv/AddTwoInts.idl
// generated code does not contain a copyright notice

#ifndef ROSBASIC_MSGS__SRV__DETAIL__ADD_TWO_INTS__STRUCT_HPP_
#define ROSBASIC_MSGS__SRV__DETAIL__ADD_TWO_INTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosbasic_msgs__srv__AddTwoInts_Request __attribute__((deprecated))
#else
# define DEPRECATED__rosbasic_msgs__srv__AddTwoInts_Request __declspec(deprecated)
#endif

namespace rosbasic_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AddTwoInts_Request_
{
  using Type = AddTwoInts_Request_<ContainerAllocator>;

  explicit AddTwoInts_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->c = 0ll;
      this->d = 0ll;
    }
  }

  explicit AddTwoInts_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->c = 0ll;
      this->d = 0ll;
    }
  }

  // field types and members
  using _c_type =
    int64_t;
  _c_type c;
  using _d_type =
    int64_t;
  _d_type d;

  // setters for named parameter idiom
  Type & set__c(
    const int64_t & _arg)
  {
    this->c = _arg;
    return *this;
  }
  Type & set__d(
    const int64_t & _arg)
  {
    this->d = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbasic_msgs__srv__AddTwoInts_Request
    std::shared_ptr<rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbasic_msgs__srv__AddTwoInts_Request
    std::shared_ptr<rosbasic_msgs::srv::AddTwoInts_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AddTwoInts_Request_ & other) const
  {
    if (this->c != other.c) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    return true;
  }
  bool operator!=(const AddTwoInts_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AddTwoInts_Request_

// alias to use template instance with default allocator
using AddTwoInts_Request =
  rosbasic_msgs::srv::AddTwoInts_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbasic_msgs


#ifndef _WIN32
# define DEPRECATED__rosbasic_msgs__srv__AddTwoInts_Response __attribute__((deprecated))
#else
# define DEPRECATED__rosbasic_msgs__srv__AddTwoInts_Response __declspec(deprecated)
#endif

namespace rosbasic_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AddTwoInts_Response_
{
  using Type = AddTwoInts_Response_<ContainerAllocator>;

  explicit AddTwoInts_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sum = 0ll;
    }
  }

  explicit AddTwoInts_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sum = 0ll;
    }
  }

  // field types and members
  using _sum_type =
    int64_t;
  _sum_type sum;

  // setters for named parameter idiom
  Type & set__sum(
    const int64_t & _arg)
  {
    this->sum = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbasic_msgs__srv__AddTwoInts_Response
    std::shared_ptr<rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbasic_msgs__srv__AddTwoInts_Response
    std::shared_ptr<rosbasic_msgs::srv::AddTwoInts_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AddTwoInts_Response_ & other) const
  {
    if (this->sum != other.sum) {
      return false;
    }
    return true;
  }
  bool operator!=(const AddTwoInts_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AddTwoInts_Response_

// alias to use template instance with default allocator
using AddTwoInts_Response =
  rosbasic_msgs::srv::AddTwoInts_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbasic_msgs

namespace rosbasic_msgs
{

namespace srv
{

struct AddTwoInts
{
  using Request = rosbasic_msgs::srv::AddTwoInts_Request;
  using Response = rosbasic_msgs::srv::AddTwoInts_Response;
};

}  // namespace srv

}  // namespace rosbasic_msgs

#endif  // ROSBASIC_MSGS__SRV__DETAIL__ADD_TWO_INTS__STRUCT_HPP_
