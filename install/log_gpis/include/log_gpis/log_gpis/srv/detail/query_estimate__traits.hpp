// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from log_gpis:srv/QueryEstimate.idl
// generated code does not contain a copyright notice

#ifndef LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__TRAITS_HPP_
#define LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "log_gpis/srv/detail/query_estimate__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace log_gpis
{

namespace srv
{

inline void to_flow_style_yaml(
  const QueryEstimate_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const QueryEstimate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const QueryEstimate_Request & msg, bool use_flow_style = false)
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

}  // namespace log_gpis

namespace rosidl_generator_traits
{

[[deprecated("use log_gpis::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const log_gpis::srv::QueryEstimate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  log_gpis::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use log_gpis::srv::to_yaml() instead")]]
inline std::string to_yaml(const log_gpis::srv::QueryEstimate_Request & msg)
{
  return log_gpis::srv::to_yaml(msg);
}

template<>
inline const char * data_type<log_gpis::srv::QueryEstimate_Request>()
{
  return "log_gpis::srv::QueryEstimate_Request";
}

template<>
inline const char * name<log_gpis::srv::QueryEstimate_Request>()
{
  return "log_gpis/srv/QueryEstimate_Request";
}

template<>
struct has_fixed_size<log_gpis::srv::QueryEstimate_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<log_gpis::srv::QueryEstimate_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<log_gpis::srv::QueryEstimate_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace log_gpis
{

namespace srv
{

inline void to_flow_style_yaml(
  const QueryEstimate_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: estimate
  {
    out << "estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.estimate, out);
    out << ", ";
  }

  // member: gradient_estimate
  {
    if (msg.gradient_estimate.size() == 0) {
      out << "gradient_estimate: []";
    } else {
      out << "gradient_estimate: [";
      size_t pending_items = msg.gradient_estimate.size();
      for (auto item : msg.gradient_estimate) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: hessian_estimate
  {
    if (msg.hessian_estimate.size() == 0) {
      out << "hessian_estimate: []";
    } else {
      out << "hessian_estimate: [";
      size_t pending_items = msg.hessian_estimate.size();
      for (auto item : msg.hessian_estimate) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const QueryEstimate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.estimate, out);
    out << "\n";
  }

  // member: gradient_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gradient_estimate.size() == 0) {
      out << "gradient_estimate: []\n";
    } else {
      out << "gradient_estimate:\n";
      for (auto item : msg.gradient_estimate) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: hessian_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.hessian_estimate.size() == 0) {
      out << "hessian_estimate: []\n";
    } else {
      out << "hessian_estimate:\n";
      for (auto item : msg.hessian_estimate) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const QueryEstimate_Response & msg, bool use_flow_style = false)
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

}  // namespace log_gpis

namespace rosidl_generator_traits
{

[[deprecated("use log_gpis::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const log_gpis::srv::QueryEstimate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  log_gpis::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use log_gpis::srv::to_yaml() instead")]]
inline std::string to_yaml(const log_gpis::srv::QueryEstimate_Response & msg)
{
  return log_gpis::srv::to_yaml(msg);
}

template<>
inline const char * data_type<log_gpis::srv::QueryEstimate_Response>()
{
  return "log_gpis::srv::QueryEstimate_Response";
}

template<>
inline const char * name<log_gpis::srv::QueryEstimate_Response>()
{
  return "log_gpis/srv/QueryEstimate_Response";
}

template<>
struct has_fixed_size<log_gpis::srv::QueryEstimate_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<log_gpis::srv::QueryEstimate_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<log_gpis::srv::QueryEstimate_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<log_gpis::srv::QueryEstimate>()
{
  return "log_gpis::srv::QueryEstimate";
}

template<>
inline const char * name<log_gpis::srv::QueryEstimate>()
{
  return "log_gpis/srv/QueryEstimate";
}

template<>
struct has_fixed_size<log_gpis::srv::QueryEstimate>
  : std::integral_constant<
    bool,
    has_fixed_size<log_gpis::srv::QueryEstimate_Request>::value &&
    has_fixed_size<log_gpis::srv::QueryEstimate_Response>::value
  >
{
};

template<>
struct has_bounded_size<log_gpis::srv::QueryEstimate>
  : std::integral_constant<
    bool,
    has_bounded_size<log_gpis::srv::QueryEstimate_Request>::value &&
    has_bounded_size<log_gpis::srv::QueryEstimate_Response>::value
  >
{
};

template<>
struct is_service<log_gpis::srv::QueryEstimate>
  : std::true_type
{
};

template<>
struct is_service_request<log_gpis::srv::QueryEstimate_Request>
  : std::true_type
{
};

template<>
struct is_service_response<log_gpis::srv::QueryEstimate_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__TRAITS_HPP_
