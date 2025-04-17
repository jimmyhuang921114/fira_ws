// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from python_moveit_interface:srv/ArmControl.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__ARM_CONTROL__STRUCT_HPP_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__ARM_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__python_moveit_interface__srv__ArmControl_Request __attribute__((deprecated))
#else
# define DEPRECATED__python_moveit_interface__srv__ArmControl_Request __declspec(deprecated)
#endif

namespace python_moveit_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ArmControl_Request_
{
  using Type = ArmControl_Request_<ContainerAllocator>;

  explicit ArmControl_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_name = "";
    }
  }

  explicit ArmControl_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_name = "";
    }
  }

  // field types and members
  using _task_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_name_type task_name;

  // setters for named parameter idiom
  Type & set__task_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__python_moveit_interface__srv__ArmControl_Request
    std::shared_ptr<python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__python_moveit_interface__srv__ArmControl_Request
    std::shared_ptr<python_moveit_interface::srv::ArmControl_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmControl_Request_ & other) const
  {
    if (this->task_name != other.task_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmControl_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmControl_Request_

// alias to use template instance with default allocator
using ArmControl_Request =
  python_moveit_interface::srv::ArmControl_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace python_moveit_interface


#ifndef _WIN32
# define DEPRECATED__python_moveit_interface__srv__ArmControl_Response __attribute__((deprecated))
#else
# define DEPRECATED__python_moveit_interface__srv__ArmControl_Response __declspec(deprecated)
#endif

namespace python_moveit_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ArmControl_Response_
{
  using Type = ArmControl_Response_<ContainerAllocator>;

  explicit ArmControl_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ArmControl_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__python_moveit_interface__srv__ArmControl_Response
    std::shared_ptr<python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__python_moveit_interface__srv__ArmControl_Response
    std::shared_ptr<python_moveit_interface::srv::ArmControl_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmControl_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmControl_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmControl_Response_

// alias to use template instance with default allocator
using ArmControl_Response =
  python_moveit_interface::srv::ArmControl_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace python_moveit_interface

namespace python_moveit_interface
{

namespace srv
{

struct ArmControl
{
  using Request = python_moveit_interface::srv::ArmControl_Request;
  using Response = python_moveit_interface::srv::ArmControl_Response;
};

}  // namespace srv

}  // namespace python_moveit_interface

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__ARM_CONTROL__STRUCT_HPP_
