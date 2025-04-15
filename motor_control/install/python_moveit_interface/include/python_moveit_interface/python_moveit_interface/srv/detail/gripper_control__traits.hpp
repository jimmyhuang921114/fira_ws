// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from python_moveit_interface:srv/GripperControl.idl
// generated code does not contain a copyright notice

#ifndef PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__GRIPPER_CONTROL__TRAITS_HPP_
#define PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__GRIPPER_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "python_moveit_interface/srv/detail/gripper_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace python_moveit_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const GripperControl_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: close
  {
    out << "close: ";
    rosidl_generator_traits::value_to_yaml(msg.close, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: close
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "close: ";
    rosidl_generator_traits::value_to_yaml(msg.close, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperControl_Request & msg, bool use_flow_style = false)
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
  const python_moveit_interface::srv::GripperControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  python_moveit_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use python_moveit_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const python_moveit_interface::srv::GripperControl_Request & msg)
{
  return python_moveit_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<python_moveit_interface::srv::GripperControl_Request>()
{
  return "python_moveit_interface::srv::GripperControl_Request";
}

template<>
inline const char * name<python_moveit_interface::srv::GripperControl_Request>()
{
  return "python_moveit_interface/srv/GripperControl_Request";
}

template<>
struct has_fixed_size<python_moveit_interface::srv::GripperControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<python_moveit_interface::srv::GripperControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<python_moveit_interface::srv::GripperControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace python_moveit_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const GripperControl_Response & msg,
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
  const GripperControl_Response & msg,
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

inline std::string to_yaml(const GripperControl_Response & msg, bool use_flow_style = false)
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
  const python_moveit_interface::srv::GripperControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  python_moveit_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use python_moveit_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const python_moveit_interface::srv::GripperControl_Response & msg)
{
  return python_moveit_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<python_moveit_interface::srv::GripperControl_Response>()
{
  return "python_moveit_interface::srv::GripperControl_Response";
}

template<>
inline const char * name<python_moveit_interface::srv::GripperControl_Response>()
{
  return "python_moveit_interface/srv/GripperControl_Response";
}

template<>
struct has_fixed_size<python_moveit_interface::srv::GripperControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<python_moveit_interface::srv::GripperControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<python_moveit_interface::srv::GripperControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<python_moveit_interface::srv::GripperControl>()
{
  return "python_moveit_interface::srv::GripperControl";
}

template<>
inline const char * name<python_moveit_interface::srv::GripperControl>()
{
  return "python_moveit_interface/srv/GripperControl";
}

template<>
struct has_fixed_size<python_moveit_interface::srv::GripperControl>
  : std::integral_constant<
    bool,
    has_fixed_size<python_moveit_interface::srv::GripperControl_Request>::value &&
    has_fixed_size<python_moveit_interface::srv::GripperControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<python_moveit_interface::srv::GripperControl>
  : std::integral_constant<
    bool,
    has_bounded_size<python_moveit_interface::srv::GripperControl_Request>::value &&
    has_bounded_size<python_moveit_interface::srv::GripperControl_Response>::value
  >
{
};

template<>
struct is_service<python_moveit_interface::srv::GripperControl>
  : std::true_type
{
};

template<>
struct is_service_request<python_moveit_interface::srv::GripperControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<python_moveit_interface::srv::GripperControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PYTHON_MOVEIT_INTERFACE__SRV__DETAIL__GRIPPER_CONTROL__TRAITS_HPP_
