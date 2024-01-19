// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from log_gpis:srv/QueryEstimate.idl
// generated code does not contain a copyright notice

#ifndef LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__STRUCT_H_
#define LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/QueryEstimate in the package log_gpis.
typedef struct log_gpis__srv__QueryEstimate_Request
{
  uint8_t structure_needs_at_least_one_member;
} log_gpis__srv__QueryEstimate_Request;

// Struct for a sequence of log_gpis__srv__QueryEstimate_Request.
typedef struct log_gpis__srv__QueryEstimate_Request__Sequence
{
  log_gpis__srv__QueryEstimate_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} log_gpis__srv__QueryEstimate_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/QueryEstimate in the package log_gpis.
typedef struct log_gpis__srv__QueryEstimate_Response
{
  double estimate;
  double gradient_estimate[3];
  double hessian_estimate[9];
} log_gpis__srv__QueryEstimate_Response;

// Struct for a sequence of log_gpis__srv__QueryEstimate_Response.
typedef struct log_gpis__srv__QueryEstimate_Response__Sequence
{
  log_gpis__srv__QueryEstimate_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} log_gpis__srv__QueryEstimate_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__STRUCT_H_
