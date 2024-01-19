// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from log_gpis:srv/QueryEstimate.idl
// generated code does not contain a copyright notice
#include "log_gpis/srv/detail/query_estimate__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "log_gpis/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "log_gpis/srv/detail/query_estimate__struct.h"
#include "log_gpis/srv/detail/query_estimate__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _QueryEstimate_Request__ros_msg_type = log_gpis__srv__QueryEstimate_Request;

static bool _QueryEstimate_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _QueryEstimate_Request__ros_msg_type * ros_message = static_cast<const _QueryEstimate_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _QueryEstimate_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _QueryEstimate_Request__ros_msg_type * ros_message = static_cast<_QueryEstimate_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_log_gpis
size_t get_serialized_size_log_gpis__srv__QueryEstimate_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _QueryEstimate_Request__ros_msg_type * ros_message = static_cast<const _QueryEstimate_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _QueryEstimate_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_log_gpis__srv__QueryEstimate_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_log_gpis
size_t max_serialized_size_log_gpis__srv__QueryEstimate_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: structure_needs_at_least_one_member
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = log_gpis__srv__QueryEstimate_Request;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _QueryEstimate_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_log_gpis__srv__QueryEstimate_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_QueryEstimate_Request = {
  "log_gpis::srv",
  "QueryEstimate_Request",
  _QueryEstimate_Request__cdr_serialize,
  _QueryEstimate_Request__cdr_deserialize,
  _QueryEstimate_Request__get_serialized_size,
  _QueryEstimate_Request__max_serialized_size
};

static rosidl_message_type_support_t _QueryEstimate_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_QueryEstimate_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, log_gpis, srv, QueryEstimate_Request)() {
  return &_QueryEstimate_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "log_gpis/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "log_gpis/srv/detail/query_estimate__struct.h"
// already included above
// #include "log_gpis/srv/detail/query_estimate__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _QueryEstimate_Response__ros_msg_type = log_gpis__srv__QueryEstimate_Response;

static bool _QueryEstimate_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _QueryEstimate_Response__ros_msg_type * ros_message = static_cast<const _QueryEstimate_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: estimate
  {
    cdr << ros_message->estimate;
  }

  // Field name: gradient_estimate
  {
    size_t size = 3;
    auto array_ptr = ros_message->gradient_estimate;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: hessian_estimate
  {
    size_t size = 9;
    auto array_ptr = ros_message->hessian_estimate;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _QueryEstimate_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _QueryEstimate_Response__ros_msg_type * ros_message = static_cast<_QueryEstimate_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: estimate
  {
    cdr >> ros_message->estimate;
  }

  // Field name: gradient_estimate
  {
    size_t size = 3;
    auto array_ptr = ros_message->gradient_estimate;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: hessian_estimate
  {
    size_t size = 9;
    auto array_ptr = ros_message->hessian_estimate;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_log_gpis
size_t get_serialized_size_log_gpis__srv__QueryEstimate_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _QueryEstimate_Response__ros_msg_type * ros_message = static_cast<const _QueryEstimate_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name estimate
  {
    size_t item_size = sizeof(ros_message->estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gradient_estimate
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->gradient_estimate;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hessian_estimate
  {
    size_t array_size = 9;
    auto array_ptr = ros_message->hessian_estimate;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _QueryEstimate_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_log_gpis__srv__QueryEstimate_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_log_gpis
size_t max_serialized_size_log_gpis__srv__QueryEstimate_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: estimate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: gradient_estimate
  {
    size_t array_size = 3;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: hessian_estimate
  {
    size_t array_size = 9;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = log_gpis__srv__QueryEstimate_Response;
    is_plain =
      (
      offsetof(DataType, hessian_estimate) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _QueryEstimate_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_log_gpis__srv__QueryEstimate_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_QueryEstimate_Response = {
  "log_gpis::srv",
  "QueryEstimate_Response",
  _QueryEstimate_Response__cdr_serialize,
  _QueryEstimate_Response__cdr_deserialize,
  _QueryEstimate_Response__get_serialized_size,
  _QueryEstimate_Response__max_serialized_size
};

static rosidl_message_type_support_t _QueryEstimate_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_QueryEstimate_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, log_gpis, srv, QueryEstimate_Response)() {
  return &_QueryEstimate_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "log_gpis/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "log_gpis/srv/query_estimate.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t QueryEstimate__callbacks = {
  "log_gpis::srv",
  "QueryEstimate",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, log_gpis, srv, QueryEstimate_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, log_gpis, srv, QueryEstimate_Response)(),
};

static rosidl_service_type_support_t QueryEstimate__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &QueryEstimate__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, log_gpis, srv, QueryEstimate)() {
  return &QueryEstimate__handle;
}

#if defined(__cplusplus)
}
#endif
