// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from map_if:srv/Command.idl
// generated code does not contain a copyright notice

#ifndef MAP_IF__SRV__DETAIL__COMMAND__TRAITS_HPP_
#define MAP_IF__SRV__DETAIL__COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "map_if/srv/detail/command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace map_if
{

namespace srv
{

inline void to_flow_style_yaml(
  const Command_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd
  {
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_Request & msg, bool use_flow_style = false)
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

}  // namespace map_if

namespace rosidl_generator_traits
{

[[deprecated("use map_if::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const map_if::srv::Command_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  map_if::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use map_if::srv::to_yaml() instead")]]
inline std::string to_yaml(const map_if::srv::Command_Request & msg)
{
  return map_if::srv::to_yaml(msg);
}

template<>
inline const char * data_type<map_if::srv::Command_Request>()
{
  return "map_if::srv::Command_Request";
}

template<>
inline const char * name<map_if::srv::Command_Request>()
{
  return "map_if/srv/Command_Request";
}

template<>
struct has_fixed_size<map_if::srv::Command_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<map_if::srv::Command_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<map_if::srv::Command_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace map_if
{

namespace srv
{

inline void to_flow_style_yaml(
  const Command_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Command_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Command_Response & msg, bool use_flow_style = false)
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

}  // namespace map_if

namespace rosidl_generator_traits
{

[[deprecated("use map_if::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const map_if::srv::Command_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  map_if::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use map_if::srv::to_yaml() instead")]]
inline std::string to_yaml(const map_if::srv::Command_Response & msg)
{
  return map_if::srv::to_yaml(msg);
}

template<>
inline const char * data_type<map_if::srv::Command_Response>()
{
  return "map_if::srv::Command_Response";
}

template<>
inline const char * name<map_if::srv::Command_Response>()
{
  return "map_if/srv/Command_Response";
}

template<>
struct has_fixed_size<map_if::srv::Command_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<map_if::srv::Command_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<map_if::srv::Command_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<map_if::srv::Command>()
{
  return "map_if::srv::Command";
}

template<>
inline const char * name<map_if::srv::Command>()
{
  return "map_if/srv/Command";
}

template<>
struct has_fixed_size<map_if::srv::Command>
  : std::integral_constant<
    bool,
    has_fixed_size<map_if::srv::Command_Request>::value &&
    has_fixed_size<map_if::srv::Command_Response>::value
  >
{
};

template<>
struct has_bounded_size<map_if::srv::Command>
  : std::integral_constant<
    bool,
    has_bounded_size<map_if::srv::Command_Request>::value &&
    has_bounded_size<map_if::srv::Command_Response>::value
  >
{
};

template<>
struct is_service<map_if::srv::Command>
  : std::true_type
{
};

template<>
struct is_service_request<map_if::srv::Command_Request>
  : std::true_type
{
};

template<>
struct is_service_response<map_if::srv::Command_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAP_IF__SRV__DETAIL__COMMAND__TRAITS_HPP_
