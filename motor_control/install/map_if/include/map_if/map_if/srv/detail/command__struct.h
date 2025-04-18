// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from map_if:srv/Command.idl
// generated code does not contain a copyright notice

#ifndef MAP_IF__SRV__DETAIL__COMMAND__STRUCT_H_
#define MAP_IF__SRV__DETAIL__COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cmd'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Command in the package map_if.
typedef struct map_if__srv__Command_Request
{
  rosidl_runtime_c__String cmd;
} map_if__srv__Command_Request;

// Struct for a sequence of map_if__srv__Command_Request.
typedef struct map_if__srv__Command_Request__Sequence
{
  map_if__srv__Command_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} map_if__srv__Command_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Command in the package map_if.
typedef struct map_if__srv__Command_Response
{
  int32_t result;
} map_if__srv__Command_Response;

// Struct for a sequence of map_if__srv__Command_Response.
typedef struct map_if__srv__Command_Response__Sequence
{
  map_if__srv__Command_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} map_if__srv__Command_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAP_IF__SRV__DETAIL__COMMAND__STRUCT_H_
