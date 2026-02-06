// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_msgs:msg/Metadata.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__METADATA__STRUCT_HPP_
#define LIDAR_MSGS__MSG__DETAIL__METADATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__lidar_msgs__msg__Metadata __attribute__((deprecated))
#else
# define DEPRECATED__lidar_msgs__msg__Metadata __declspec(deprecated)
#endif

namespace lidar_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Metadata_
{
  using Type = Metadata_<ContainerAllocator>;

  explicit Metadata_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->computer_ip = "";
      this->lidar_ip = "";
      this->imu_port = 0;
      this->lidar_port = 0;
    }
  }

  explicit Metadata_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : computer_ip(_alloc),
    lidar_ip(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->computer_ip = "";
      this->lidar_ip = "";
      this->imu_port = 0;
      this->lidar_port = 0;
    }
  }

  // field types and members
  using _computer_ip_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _computer_ip_type computer_ip;
  using _lidar_ip_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _lidar_ip_type lidar_ip;
  using _imu_port_type =
    int8_t;
  _imu_port_type imu_port;
  using _lidar_port_type =
    int8_t;
  _lidar_port_type lidar_port;

  // setters for named parameter idiom
  Type & set__computer_ip(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->computer_ip = _arg;
    return *this;
  }
  Type & set__lidar_ip(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->lidar_ip = _arg;
    return *this;
  }
  Type & set__imu_port(
    const int8_t & _arg)
  {
    this->imu_port = _arg;
    return *this;
  }
  Type & set__lidar_port(
    const int8_t & _arg)
  {
    this->lidar_port = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_msgs::msg::Metadata_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_msgs::msg::Metadata_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_msgs::msg::Metadata_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_msgs::msg::Metadata_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_msgs::msg::Metadata_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_msgs::msg::Metadata_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_msgs::msg::Metadata_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_msgs::msg::Metadata_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_msgs::msg::Metadata_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_msgs::msg::Metadata_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_msgs__msg__Metadata
    std::shared_ptr<lidar_msgs::msg::Metadata_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_msgs__msg__Metadata
    std::shared_ptr<lidar_msgs::msg::Metadata_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Metadata_ & other) const
  {
    if (this->computer_ip != other.computer_ip) {
      return false;
    }
    if (this->lidar_ip != other.lidar_ip) {
      return false;
    }
    if (this->imu_port != other.imu_port) {
      return false;
    }
    if (this->lidar_port != other.lidar_port) {
      return false;
    }
    return true;
  }
  bool operator!=(const Metadata_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Metadata_

// alias to use template instance with default allocator
using Metadata =
  lidar_msgs::msg::Metadata_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_msgs

#endif  // LIDAR_MSGS__MSG__DETAIL__METADATA__STRUCT_HPP_
