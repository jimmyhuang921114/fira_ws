// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from python_moveit_interface:srv/PoseRequest.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__STRUCT_HPP_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__python_moveit_interface__srv__PoseRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__python_moveit_interface__srv__PoseRequest_Request __declspec(deprecated)
#endif

namespace python_moveit_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PoseRequest_Request_
{
  using Type = PoseRequest_Request_<ContainerAllocator>;

  explicit PoseRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = "";
    }
  }

  explicit PoseRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_alloc, _init),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = "";
    }
  }

  // field types and members
  using _target_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _target_pose_type target_pose;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__target_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->target_pose = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__python_moveit_interface__srv__PoseRequest_Request
    std::shared_ptr<python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__python_moveit_interface__srv__PoseRequest_Request
    std::shared_ptr<python_moveit_interface::srv::PoseRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseRequest_Request_ & other) const
  {
    if (this->target_pose != other.target_pose) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseRequest_Request_

// alias to use template instance with default allocator
using PoseRequest_Request =
  python_moveit_interface::srv::PoseRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace python_moveit_interface


#ifndef _WIN32
# define DEPRECATED__python_moveit_interface__srv__PoseRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__python_moveit_interface__srv__PoseRequest_Response __declspec(deprecated)
#endif

namespace python_moveit_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PoseRequest_Response_
{
  using Type = PoseRequest_Response_<ContainerAllocator>;

  explicit PoseRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit PoseRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__python_moveit_interface__srv__PoseRequest_Response
    std::shared_ptr<python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__python_moveit_interface__srv__PoseRequest_Response
    std::shared_ptr<python_moveit_interface::srv::PoseRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PoseRequest_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PoseRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PoseRequest_Response_

// alias to use template instance with default allocator
using PoseRequest_Response =
  python_moveit_interface::srv::PoseRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace python_moveit_interface

namespace python_moveit_interface
{

namespace srv
{

struct PoseRequest
{
  using Request = python_moveit_interface::srv::PoseRequest_Request;
  using Response = python_moveit_interface::srv::PoseRequest_Response;
};

}  // namespace srv

}  // namespace python_moveit_interface

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__STRUCT_HPP_
