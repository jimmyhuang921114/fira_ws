// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from python_moveit_interface:srv/DetectPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "python_moveit_interface/srv/detail/detect_pose__rosidl_typesupport_introspection_c.h"
#include "python_moveit_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "python_moveit_interface/srv/detail/detect_pose__functions.h"
#include "python_moveit_interface/srv/detail/detect_pose__struct.h"


// Include directives for member types
// Member `target_pose`
#include "geometry_msgs/msg/pose.h"
// Member `target_pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  python_moveit_interface__srv__DetectPose_Request__init(message_memory);
}

void python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_fini_function(void * message_memory)
{
  python_moveit_interface__srv__DetectPose_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_message_member_array[1] = {
  {
    "target_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(python_moveit_interface__srv__DetectPose_Request, target_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_message_members = {
  "python_moveit_interface__srv",  // message namespace
  "DetectPose_Request",  // message name
  1,  // number of fields
  sizeof(python_moveit_interface__srv__DetectPose_Request),
  python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_message_member_array,  // message members
  python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_message_type_support_handle = {
  0,
  &python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_python_moveit_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, python_moveit_interface, srv, DetectPose_Request)() {
  python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_message_type_support_handle.typesupport_identifier) {
    python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &python_moveit_interface__srv__DetectPose_Request__rosidl_typesupport_introspection_c__DetectPose_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "python_moveit_interface/srv/detail/detect_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "python_moveit_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "python_moveit_interface/srv/detail/detect_pose__functions.h"
// already included above
// #include "python_moveit_interface/srv/detail/detect_pose__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  python_moveit_interface__srv__DetectPose_Response__init(message_memory);
}

void python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_fini_function(void * message_memory)
{
  python_moveit_interface__srv__DetectPose_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(python_moveit_interface__srv__DetectPose_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(python_moveit_interface__srv__DetectPose_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_message_members = {
  "python_moveit_interface__srv",  // message namespace
  "DetectPose_Response",  // message name
  2,  // number of fields
  sizeof(python_moveit_interface__srv__DetectPose_Response),
  python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_message_member_array,  // message members
  python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_message_type_support_handle = {
  0,
  &python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_python_moveit_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, python_moveit_interface, srv, DetectPose_Response)() {
  if (!python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_message_type_support_handle.typesupport_identifier) {
    python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &python_moveit_interface__srv__DetectPose_Response__rosidl_typesupport_introspection_c__DetectPose_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "python_moveit_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "python_moveit_interface/srv/detail/detect_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers python_moveit_interface__srv__detail__detect_pose__rosidl_typesupport_introspection_c__DetectPose_service_members = {
  "python_moveit_interface__srv",  // service namespace
  "DetectPose",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // python_moveit_interface__srv__detail__detect_pose__rosidl_typesupport_introspection_c__DetectPose_Request_message_type_support_handle,
  NULL  // response message
  // python_moveit_interface__srv__detail__detect_pose__rosidl_typesupport_introspection_c__DetectPose_Response_message_type_support_handle
};

static rosidl_service_type_support_t python_moveit_interface__srv__detail__detect_pose__rosidl_typesupport_introspection_c__DetectPose_service_type_support_handle = {
  0,
  &python_moveit_interface__srv__detail__detect_pose__rosidl_typesupport_introspection_c__DetectPose_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, python_moveit_interface, srv, DetectPose_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, python_moveit_interface, srv, DetectPose_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_python_moveit_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, python_moveit_interface, srv, DetectPose)() {
  if (!python_moveit_interface__srv__detail__detect_pose__rosidl_typesupport_introspection_c__DetectPose_service_type_support_handle.typesupport_identifier) {
    python_moveit_interface__srv__detail__detect_pose__rosidl_typesupport_introspection_c__DetectPose_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)python_moveit_interface__srv__detail__detect_pose__rosidl_typesupport_introspection_c__DetectPose_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, python_moveit_interface, srv, DetectPose_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, python_moveit_interface, srv, DetectPose_Response)()->data;
  }

  return &python_moveit_interface__srv__detail__detect_pose__rosidl_typesupport_introspection_c__DetectPose_service_type_support_handle;
}
