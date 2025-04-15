// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from python_moveit_interface:srv/GripperControl.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__GRIPPER_CONTROL__BUILDER_HPP_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__GRIPPER_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "python_moveit_interface/srv/detail/gripper_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace python_moveit_interface
{

namespace srv
{

namespace builder
{

class Init_GripperControl_Request_close
{
public:
  Init_GripperControl_Request_close()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::python_moveit_interface::srv::GripperControl_Request close(::python_moveit_interface::srv::GripperControl_Request::_close_type arg)
  {
    msg_.close = std::move(arg);
    return std::move(msg_);
  }

private:
  ::python_moveit_interface::srv::GripperControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::python_moveit_interface::srv::GripperControl_Request>()
{
  return python_moveit_interface::srv::builder::Init_GripperControl_Request_close();
}

}  // namespace python_moveit_interface


namespace python_moveit_interface
{

namespace srv
{

namespace builder
{

class Init_GripperControl_Response_message
{
public:
  explicit Init_GripperControl_Response_message(::python_moveit_interface::srv::GripperControl_Response & msg)
  : msg_(msg)
  {}
  ::python_moveit_interface::srv::GripperControl_Response message(::python_moveit_interface::srv::GripperControl_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::python_moveit_interface::srv::GripperControl_Response msg_;
};

class Init_GripperControl_Response_success
{
public:
  Init_GripperControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperControl_Response_message success(::python_moveit_interface::srv::GripperControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GripperControl_Response_message(msg_);
  }

private:
  ::python_moveit_interface::srv::GripperControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::python_moveit_interface::srv::GripperControl_Response>()
{
  return python_moveit_interface::srv::builder::Init_GripperControl_Response_success();
}

}  // namespace python_moveit_interface

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__GRIPPER_CONTROL__BUILDER_HPP_
