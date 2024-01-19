// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from log_gpis:srv/QueryEstimate.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "log_gpis/srv/detail/query_estimate__struct.h"
#include "log_gpis/srv/detail/query_estimate__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace log_gpis
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _QueryEstimate_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _QueryEstimate_Request_type_support_ids_t;

static const _QueryEstimate_Request_type_support_ids_t _QueryEstimate_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _QueryEstimate_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _QueryEstimate_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _QueryEstimate_Request_type_support_symbol_names_t _QueryEstimate_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, log_gpis, srv, QueryEstimate_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, log_gpis, srv, QueryEstimate_Request)),
  }
};

typedef struct _QueryEstimate_Request_type_support_data_t
{
  void * data[2];
} _QueryEstimate_Request_type_support_data_t;

static _QueryEstimate_Request_type_support_data_t _QueryEstimate_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _QueryEstimate_Request_message_typesupport_map = {
  2,
  "log_gpis",
  &_QueryEstimate_Request_message_typesupport_ids.typesupport_identifier[0],
  &_QueryEstimate_Request_message_typesupport_symbol_names.symbol_name[0],
  &_QueryEstimate_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t QueryEstimate_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_QueryEstimate_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace log_gpis

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, log_gpis, srv, QueryEstimate_Request)() {
  return &::log_gpis::srv::rosidl_typesupport_c::QueryEstimate_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "log_gpis/srv/detail/query_estimate__struct.h"
// already included above
// #include "log_gpis/srv/detail/query_estimate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace log_gpis
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _QueryEstimate_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _QueryEstimate_Response_type_support_ids_t;

static const _QueryEstimate_Response_type_support_ids_t _QueryEstimate_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _QueryEstimate_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _QueryEstimate_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _QueryEstimate_Response_type_support_symbol_names_t _QueryEstimate_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, log_gpis, srv, QueryEstimate_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, log_gpis, srv, QueryEstimate_Response)),
  }
};

typedef struct _QueryEstimate_Response_type_support_data_t
{
  void * data[2];
} _QueryEstimate_Response_type_support_data_t;

static _QueryEstimate_Response_type_support_data_t _QueryEstimate_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _QueryEstimate_Response_message_typesupport_map = {
  2,
  "log_gpis",
  &_QueryEstimate_Response_message_typesupport_ids.typesupport_identifier[0],
  &_QueryEstimate_Response_message_typesupport_symbol_names.symbol_name[0],
  &_QueryEstimate_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t QueryEstimate_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_QueryEstimate_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace log_gpis

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, log_gpis, srv, QueryEstimate_Response)() {
  return &::log_gpis::srv::rosidl_typesupport_c::QueryEstimate_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "log_gpis/srv/detail/query_estimate__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace log_gpis
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _QueryEstimate_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _QueryEstimate_type_support_ids_t;

static const _QueryEstimate_type_support_ids_t _QueryEstimate_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _QueryEstimate_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _QueryEstimate_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _QueryEstimate_type_support_symbol_names_t _QueryEstimate_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, log_gpis, srv, QueryEstimate)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, log_gpis, srv, QueryEstimate)),
  }
};

typedef struct _QueryEstimate_type_support_data_t
{
  void * data[2];
} _QueryEstimate_type_support_data_t;

static _QueryEstimate_type_support_data_t _QueryEstimate_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _QueryEstimate_service_typesupport_map = {
  2,
  "log_gpis",
  &_QueryEstimate_service_typesupport_ids.typesupport_identifier[0],
  &_QueryEstimate_service_typesupport_symbol_names.symbol_name[0],
  &_QueryEstimate_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t QueryEstimate_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_QueryEstimate_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace log_gpis

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, log_gpis, srv, QueryEstimate)() {
  return &::log_gpis::srv::rosidl_typesupport_c::QueryEstimate_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
