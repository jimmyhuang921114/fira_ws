// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from python_moveit_interface:srv/PoseRequest.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__STRUCT_H_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PoseRequest in the package python_moveit_interface.
typedef struct python_moveit_interface__srv__PoseRequest_Request
{
  /// 目標點位
  geometry_msgs__msg__Pose target_pose;
  rosidl_runtime_c__String message;
} python_moveit_interface__srv__PoseRequest_Request;

// Struct for a sequence of python_moveit_interface__srv__PoseRequest_Request.
typedef struct python_moveit_interface__srv__PoseRequest_Request__Sequence
{
  python_moveit_interface__srv__PoseRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} python_moveit_interface__srv__PoseRequest_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/PoseRequest in the package python_moveit_interface.
typedef struct python_moveit_interface__srv__PoseRequest_Response
{
  bool success;
  rosidl_runtime_c__String message;
} python_moveit_interface__srv__PoseRequest_Response;

// Struct for a sequence of python_moveit_interface__srv__PoseRequest_Response.
typedef struct python_moveit_interface__srv__PoseRequest_Response__Sequence
{
  python_moveit_interface__srv__PoseRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} python_moveit_interface__srv__PoseRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__STRUCT_H_
