// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from python_moveit_interface:srv/ArmControl.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__ARM_CONTROL__STRUCT_H_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__ARM_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'task_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ArmControl in the package python_moveit_interface.
typedef struct python_moveit_interface__srv__ArmControl_Request
{
  /// 指定命名姿態，例如 put_1、home 等
  rosidl_runtime_c__String task_name;
} python_moveit_interface__srv__ArmControl_Request;

// Struct for a sequence of python_moveit_interface__srv__ArmControl_Request.
typedef struct python_moveit_interface__srv__ArmControl_Request__Sequence
{
  python_moveit_interface__srv__ArmControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} python_moveit_interface__srv__ArmControl_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ArmControl in the package python_moveit_interface.
typedef struct python_moveit_interface__srv__ArmControl_Response
{
  /// 是否成功完成
  bool success;
  /// 回傳訊息
  rosidl_runtime_c__String message;
} python_moveit_interface__srv__ArmControl_Response;

// Struct for a sequence of python_moveit_interface__srv__ArmControl_Response.
typedef struct python_moveit_interface__srv__ArmControl_Response__Sequence
{
  python_moveit_interface__srv__ArmControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} python_moveit_interface__srv__ArmControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__ARM_CONTROL__STRUCT_H_
