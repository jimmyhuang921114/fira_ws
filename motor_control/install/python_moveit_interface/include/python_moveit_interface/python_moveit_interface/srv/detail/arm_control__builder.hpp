// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from python_moveit_interface:srv/ArmControl.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__ARM_CONTROL__BUILDER_HPP_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__ARM_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "python_moveit_interface/srv/detail/arm_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace python_moveit_interface
{

namespace srv
{

namespace builder
{

class Init_ArmControl_Request_task_name
{
public:
  Init_ArmControl_Request_task_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::python_moveit_interface::srv::ArmControl_Request task_name(::python_moveit_interface::srv::ArmControl_Request::_task_name_type arg)
  {
    msg_.task_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::python_moveit_interface::srv::ArmControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::python_moveit_interface::srv::ArmControl_Request>()
{
  return python_moveit_interface::srv::builder::Init_ArmControl_Request_task_name();
}

}  // namespace python_moveit_interface


namespace python_moveit_interface
{

namespace srv
{

namespace builder
{

class Init_ArmControl_Response_message
{
public:
  explicit Init_ArmControl_Response_message(::python_moveit_interface::srv::ArmControl_Response & msg)
  : msg_(msg)
  {}
  ::python_moveit_interface::srv::ArmControl_Response message(::python_moveit_interface::srv::ArmControl_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::python_moveit_interface::srv::ArmControl_Response msg_;
};

class Init_ArmControl_Response_success
{
public:
  Init_ArmControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmControl_Response_message success(::python_moveit_interface::srv::ArmControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ArmControl_Response_message(msg_);
  }

private:
  ::python_moveit_interface::srv::ArmControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::python_moveit_interface::srv::ArmControl_Response>()
{
  return python_moveit_interface::srv::builder::Init_ArmControl_Response_success();
}

}  // namespace python_moveit_interface

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__ARM_CONTROL__BUILDER_HPP_
