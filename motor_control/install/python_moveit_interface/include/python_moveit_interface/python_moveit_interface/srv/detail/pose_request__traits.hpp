// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from python_moveit_interface:srv/PoseRequest.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__TRAITS_HPP_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "python_moveit_interface/srv/detail/pose_request__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace python_moveit_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const PoseRequest_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_pose
  {
    out << "target_pose: ";
    to_flow_style_yaml(msg.target_pose, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PoseRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pose:\n";
    to_block_style_yaml(msg.target_pose, out, indentation + 2);
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PoseRequest_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace python_moveit_interface

namespace rosidl_generator_traits
{

[[deprecated("use python_moveit_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const python_moveit_interface::srv::PoseRequest_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  python_moveit_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use python_moveit_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const python_moveit_interface::srv::PoseRequest_Request & msg)
{
  return python_moveit_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<python_moveit_interface::srv::PoseRequest_Request>()
{
  return "python_moveit_interface::srv::PoseRequest_Request";
}

template<>
inline const char * name<python_moveit_interface::srv::PoseRequest_Request>()
{
  return "python_moveit_interface/srv/PoseRequest_Request";
}

template<>
struct has_fixed_size<python_moveit_interface::srv::PoseRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<python_moveit_interface::srv::PoseRequest_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<python_moveit_interface::srv::PoseRequest_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace python_moveit_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const PoseRequest_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PoseRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PoseRequest_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace python_moveit_interface

namespace rosidl_generator_traits
{

[[deprecated("use python_moveit_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const python_moveit_interface::srv::PoseRequest_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  python_moveit_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use python_moveit_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const python_moveit_interface::srv::PoseRequest_Response & msg)
{
  return python_moveit_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<python_moveit_interface::srv::PoseRequest_Response>()
{
  return "python_moveit_interface::srv::PoseRequest_Response";
}

template<>
inline const char * name<python_moveit_interface::srv::PoseRequest_Response>()
{
  return "python_moveit_interface/srv/PoseRequest_Response";
}

template<>
struct has_fixed_size<python_moveit_interface::srv::PoseRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<python_moveit_interface::srv::PoseRequest_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<python_moveit_interface::srv::PoseRequest_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<python_moveit_interface::srv::PoseRequest>()
{
  return "python_moveit_interface::srv::PoseRequest";
}

template<>
inline const char * name<python_moveit_interface::srv::PoseRequest>()
{
  return "python_moveit_interface/srv/PoseRequest";
}

template<>
struct has_fixed_size<python_moveit_interface::srv::PoseRequest>
  : std::integral_constant<
    bool,
    has_fixed_size<python_moveit_interface::srv::PoseRequest_Request>::value &&
    has_fixed_size<python_moveit_interface::srv::PoseRequest_Response>::value
  >
{
};

template<>
struct has_bounded_size<python_moveit_interface::srv::PoseRequest>
  : std::integral_constant<
    bool,
    has_bounded_size<python_moveit_interface::srv::PoseRequest_Request>::value &&
    has_bounded_size<python_moveit_interface::srv::PoseRequest_Response>::value
  >
{
};

template<>
struct is_service<python_moveit_interface::srv::PoseRequest>
  : std::true_type
{
};

template<>
struct is_service_request<python_moveit_interface::srv::PoseRequest_Request>
  : std::true_type
{
};

template<>
struct is_service_response<python_moveit_interface::srv::PoseRequest_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__POSE_REQUEST__TRAITS_HPP_
