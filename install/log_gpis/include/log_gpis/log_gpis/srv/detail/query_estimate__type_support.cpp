// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from log_gpis:srv/QueryEstimate.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "log_gpis/srv/detail/query_estimate__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace log_gpis
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void QueryEstimate_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) log_gpis::srv::QueryEstimate_Request(_init);
}

void QueryEstimate_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<log_gpis::srv::QueryEstimate_Request *>(message_memory);
  typed_message->~QueryEstimate_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember QueryEstimate_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(log_gpis::srv::QueryEstimate_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers QueryEstimate_Request_message_members = {
  "log_gpis::srv",  // message namespace
  "QueryEstimate_Request",  // message name
  1,  // number of fields
  sizeof(log_gpis::srv::QueryEstimate_Request),
  QueryEstimate_Request_message_member_array,  // message members
  QueryEstimate_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  QueryEstimate_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t QueryEstimate_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &QueryEstimate_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace log_gpis


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<log_gpis::srv::QueryEstimate_Request>()
{
  return &::log_gpis::srv::rosidl_typesupport_introspection_cpp::QueryEstimate_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, log_gpis, srv, QueryEstimate_Request)() {
  return &::log_gpis::srv::rosidl_typesupport_introspection_cpp::QueryEstimate_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "log_gpis/srv/detail/query_estimate__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace log_gpis
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void QueryEstimate_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) log_gpis::srv::QueryEstimate_Response(_init);
}

void QueryEstimate_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<log_gpis::srv::QueryEstimate_Response *>(message_memory);
  typed_message->~QueryEstimate_Response();
}

size_t size_function__QueryEstimate_Response__gradient_estimate(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__QueryEstimate_Response__gradient_estimate(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__QueryEstimate_Response__gradient_estimate(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__QueryEstimate_Response__gradient_estimate(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__QueryEstimate_Response__gradient_estimate(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__QueryEstimate_Response__gradient_estimate(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__QueryEstimate_Response__gradient_estimate(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__QueryEstimate_Response__hessian_estimate(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__QueryEstimate_Response__hessian_estimate(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__QueryEstimate_Response__hessian_estimate(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__QueryEstimate_Response__hessian_estimate(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__QueryEstimate_Response__hessian_estimate(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__QueryEstimate_Response__hessian_estimate(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__QueryEstimate_Response__hessian_estimate(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember QueryEstimate_Response_message_member_array[3] = {
  {
    "estimate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(log_gpis::srv::QueryEstimate_Response, estimate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gradient_estimate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(log_gpis::srv::QueryEstimate_Response, gradient_estimate),  // bytes offset in struct
    nullptr,  // default value
    size_function__QueryEstimate_Response__gradient_estimate,  // size() function pointer
    get_const_function__QueryEstimate_Response__gradient_estimate,  // get_const(index) function pointer
    get_function__QueryEstimate_Response__gradient_estimate,  // get(index) function pointer
    fetch_function__QueryEstimate_Response__gradient_estimate,  // fetch(index, &value) function pointer
    assign_function__QueryEstimate_Response__gradient_estimate,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "hessian_estimate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(log_gpis::srv::QueryEstimate_Response, hessian_estimate),  // bytes offset in struct
    nullptr,  // default value
    size_function__QueryEstimate_Response__hessian_estimate,  // size() function pointer
    get_const_function__QueryEstimate_Response__hessian_estimate,  // get_const(index) function pointer
    get_function__QueryEstimate_Response__hessian_estimate,  // get(index) function pointer
    fetch_function__QueryEstimate_Response__hessian_estimate,  // fetch(index, &value) function pointer
    assign_function__QueryEstimate_Response__hessian_estimate,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers QueryEstimate_Response_message_members = {
  "log_gpis::srv",  // message namespace
  "QueryEstimate_Response",  // message name
  3,  // number of fields
  sizeof(log_gpis::srv::QueryEstimate_Response),
  QueryEstimate_Response_message_member_array,  // message members
  QueryEstimate_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  QueryEstimate_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t QueryEstimate_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &QueryEstimate_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace log_gpis


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<log_gpis::srv::QueryEstimate_Response>()
{
  return &::log_gpis::srv::rosidl_typesupport_introspection_cpp::QueryEstimate_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, log_gpis, srv, QueryEstimate_Response)() {
  return &::log_gpis::srv::rosidl_typesupport_introspection_cpp::QueryEstimate_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "log_gpis/srv/detail/query_estimate__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace log_gpis
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers QueryEstimate_service_members = {
  "log_gpis::srv",  // service namespace
  "QueryEstimate",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<log_gpis::srv::QueryEstimate>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t QueryEstimate_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &QueryEstimate_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace log_gpis


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<log_gpis::srv::QueryEstimate>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::log_gpis::srv::rosidl_typesupport_introspection_cpp::QueryEstimate_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::log_gpis::srv::QueryEstimate_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::log_gpis::srv::QueryEstimate_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, log_gpis, srv, QueryEstimate)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<log_gpis::srv::QueryEstimate>();
}

#ifdef __cplusplus
}
#endif
