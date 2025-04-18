// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from python_moveit_interface:srv/DetectPose.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__DETECT_POSE__STRUCT_H_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__DETECT_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/DetectPose in the package python_moveit_interface.
typedef struct python_moveit_interface__srv__DetectPose_Request
{
  uint8_t structure_needs_at_least_one_member;
} python_moveit_interface__srv__DetectPose_Request;

// Struct for a sequence of python_moveit_interface__srv__DetectPose_Request.
typedef struct python_moveit_interface__srv__DetectPose_Request__Sequence
{
  python_moveit_interface__srv__DetectPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} python_moveit_interface__srv__DetectPose_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/DetectPose in the package python_moveit_interface.
typedef struct python_moveit_interface__srv__DetectPose_Response
{
  bool success;
  rosidl_runtime_c__String message;
} python_moveit_interface__srv__DetectPose_Response;

// Struct for a sequence of python_moveit_interface__srv__DetectPose_Response.
typedef struct python_moveit_interface__srv__DetectPose_Response__Sequence
{
  python_moveit_interface__srv__DetectPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} python_moveit_interface__srv__DetectPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__DETECT_POSE__STRUCT_H_
