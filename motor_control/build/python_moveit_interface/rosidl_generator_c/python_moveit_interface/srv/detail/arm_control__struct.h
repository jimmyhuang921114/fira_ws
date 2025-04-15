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
// Member 'target_zone'
// Member 'named_pose'
#include "rosidl_runtime_c/string.h"
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/ArmControl in the package python_moveit_interface.
typedef struct python_moveit_interface__srv__ArmControl_Request
{
  /// 指定命名姿態，例如 put_1、home 等
  rosidl_runtime_c__String task_name;
  /// 執行抓取流程
  bool trigger_grab_flow;
  /// 執行放置流程
  bool trigger_place_flow;
  /// 指定抓取或放置的區域（例如 grab_a、put_left）
  rosidl_runtime_c__String target_zone;
  /// 可選：給定實際三維目標位置
  geometry_msgs__msg__Pose target_pose;
  /// 可選：移動至指定 named pose
  rosidl_runtime_c__String named_pose;
  /// 控制夾爪關閉/開啟
  bool gripper_close;
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
