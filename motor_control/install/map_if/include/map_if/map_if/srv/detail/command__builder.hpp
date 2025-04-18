// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from map_if:srv/Command.idl
// generated code does not contain a copyright notice

#ifndef MAP_IF__SRV__DETAIL__COMMAND__BUILDER_HPP_
#define MAP_IF__SRV__DETAIL__COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "map_if/srv/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace map_if
{

namespace srv
{

namespace builder
{

class Init_Command_Request_cmd
{
public:
  Init_Command_Request_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::map_if::srv::Command_Request cmd(::map_if::srv::Command_Request::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::map_if::srv::Command_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::map_if::srv::Command_Request>()
{
  return map_if::srv::builder::Init_Command_Request_cmd();
}

}  // namespace map_if


namespace map_if
{

namespace srv
{

namespace builder
{

class Init_Command_Response_result
{
public:
  Init_Command_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::map_if::srv::Command_Response result(::map_if::srv::Command_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::map_if::srv::Command_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::map_if::srv::Command_Response>()
{
  return map_if::srv::builder::Init_Command_Response_result();
}

}  // namespace map_if

#endif  // MAP_IF__SRV__DETAIL__COMMAND__BUILDER_HPP_
