// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from python_moveit_interface:srv/PoseRequest.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__BUILDER_HPP_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "python_moveit_interface/srv/detail/pose_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace python_moveit_interface
{

namespace srv
{

namespace builder
{

class Init_PoseRequest_Request_message
{
public:
  explicit Init_PoseRequest_Request_message(::python_moveit_interface::srv::PoseRequest_Request & msg)
  : msg_(msg)
  {}
  ::python_moveit_interface::srv::PoseRequest_Request message(::python_moveit_interface::srv::PoseRequest_Request::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::python_moveit_interface::srv::PoseRequest_Request msg_;
};

class Init_PoseRequest_Request_target_pose
{
public:
  Init_PoseRequest_Request_target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseRequest_Request_message target_pose(::python_moveit_interface::srv::PoseRequest_Request::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_PoseRequest_Request_message(msg_);
  }

private:
  ::python_moveit_interface::srv::PoseRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::python_moveit_interface::srv::PoseRequest_Request>()
{
  return python_moveit_interface::srv::builder::Init_PoseRequest_Request_target_pose();
}

}  // namespace python_moveit_interface


namespace python_moveit_interface
{

namespace srv
{

namespace builder
{

class Init_PoseRequest_Response_message
{
public:
  explicit Init_PoseRequest_Response_message(::python_moveit_interface::srv::PoseRequest_Response & msg)
  : msg_(msg)
  {}
  ::python_moveit_interface::srv::PoseRequest_Response message(::python_moveit_interface::srv::PoseRequest_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::python_moveit_interface::srv::PoseRequest_Response msg_;
};

class Init_PoseRequest_Response_success
{
public:
  Init_PoseRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseRequest_Response_message success(::python_moveit_interface::srv::PoseRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PoseRequest_Response_message(msg_);
  }

private:
  ::python_moveit_interface::srv::PoseRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::python_moveit_interface::srv::PoseRequest_Response>()
{
  return python_moveit_interface::srv::builder::Init_PoseRequest_Response_success();
}

}  // namespace python_moveit_interface

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__BUILDER_HPP_
