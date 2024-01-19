// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from log_gpis:srv/QueryEstimate.idl
// generated code does not contain a copyright notice

#ifndef LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__BUILDER_HPP_
#define LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "log_gpis/srv/detail/query_estimate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace log_gpis
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::log_gpis::srv::QueryEstimate_Request>()
{
  return ::log_gpis::srv::QueryEstimate_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace log_gpis


namespace log_gpis
{

namespace srv
{

namespace builder
{

class Init_QueryEstimate_Response_hessian_estimate
{
public:
  explicit Init_QueryEstimate_Response_hessian_estimate(::log_gpis::srv::QueryEstimate_Response & msg)
  : msg_(msg)
  {}
  ::log_gpis::srv::QueryEstimate_Response hessian_estimate(::log_gpis::srv::QueryEstimate_Response::_hessian_estimate_type arg)
  {
    msg_.hessian_estimate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::log_gpis::srv::QueryEstimate_Response msg_;
};

class Init_QueryEstimate_Response_gradient_estimate
{
public:
  explicit Init_QueryEstimate_Response_gradient_estimate(::log_gpis::srv::QueryEstimate_Response & msg)
  : msg_(msg)
  {}
  Init_QueryEstimate_Response_hessian_estimate gradient_estimate(::log_gpis::srv::QueryEstimate_Response::_gradient_estimate_type arg)
  {
    msg_.gradient_estimate = std::move(arg);
    return Init_QueryEstimate_Response_hessian_estimate(msg_);
  }

private:
  ::log_gpis::srv::QueryEstimate_Response msg_;
};

class Init_QueryEstimate_Response_estimate
{
public:
  Init_QueryEstimate_Response_estimate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_QueryEstimate_Response_gradient_estimate estimate(::log_gpis::srv::QueryEstimate_Response::_estimate_type arg)
  {
    msg_.estimate = std::move(arg);
    return Init_QueryEstimate_Response_gradient_estimate(msg_);
  }

private:
  ::log_gpis::srv::QueryEstimate_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::log_gpis::srv::QueryEstimate_Response>()
{
  return log_gpis::srv::builder::Init_QueryEstimate_Response_estimate();
}

}  // namespace log_gpis

#endif  // LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__BUILDER_HPP_
