// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from log_gpis:srv/QueryEstimate.idl
// generated code does not contain a copyright notice

#ifndef LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__STRUCT_HPP_
#define LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__log_gpis__srv__QueryEstimate_Request __attribute__((deprecated))
#else
# define DEPRECATED__log_gpis__srv__QueryEstimate_Request __declspec(deprecated)
#endif

namespace log_gpis
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct QueryEstimate_Request_
{
  using Type = QueryEstimate_Request_<ContainerAllocator>;

  explicit QueryEstimate_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit QueryEstimate_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    log_gpis::srv::QueryEstimate_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const log_gpis::srv::QueryEstimate_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<log_gpis::srv::QueryEstimate_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<log_gpis::srv::QueryEstimate_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      log_gpis::srv::QueryEstimate_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<log_gpis::srv::QueryEstimate_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      log_gpis::srv::QueryEstimate_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<log_gpis::srv::QueryEstimate_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<log_gpis::srv::QueryEstimate_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<log_gpis::srv::QueryEstimate_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__log_gpis__srv__QueryEstimate_Request
    std::shared_ptr<log_gpis::srv::QueryEstimate_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__log_gpis__srv__QueryEstimate_Request
    std::shared_ptr<log_gpis::srv::QueryEstimate_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QueryEstimate_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const QueryEstimate_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QueryEstimate_Request_

// alias to use template instance with default allocator
using QueryEstimate_Request =
  log_gpis::srv::QueryEstimate_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace log_gpis


#ifndef _WIN32
# define DEPRECATED__log_gpis__srv__QueryEstimate_Response __attribute__((deprecated))
#else
# define DEPRECATED__log_gpis__srv__QueryEstimate_Response __declspec(deprecated)
#endif

namespace log_gpis
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct QueryEstimate_Response_
{
  using Type = QueryEstimate_Response_<ContainerAllocator>;

  explicit QueryEstimate_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->estimate = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->gradient_estimate.begin(), this->gradient_estimate.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->hessian_estimate.begin(), this->hessian_estimate.end(), 0.0);
    }
  }

  explicit QueryEstimate_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : gradient_estimate(_alloc),
    hessian_estimate(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->estimate = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->gradient_estimate.begin(), this->gradient_estimate.end(), 0.0);
      std::fill<typename std::array<double, 9>::iterator, double>(this->hessian_estimate.begin(), this->hessian_estimate.end(), 0.0);
    }
  }

  // field types and members
  using _estimate_type =
    double;
  _estimate_type estimate;
  using _gradient_estimate_type =
    std::array<double, 3>;
  _gradient_estimate_type gradient_estimate;
  using _hessian_estimate_type =
    std::array<double, 9>;
  _hessian_estimate_type hessian_estimate;

  // setters for named parameter idiom
  Type & set__estimate(
    const double & _arg)
  {
    this->estimate = _arg;
    return *this;
  }
  Type & set__gradient_estimate(
    const std::array<double, 3> & _arg)
  {
    this->gradient_estimate = _arg;
    return *this;
  }
  Type & set__hessian_estimate(
    const std::array<double, 9> & _arg)
  {
    this->hessian_estimate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    log_gpis::srv::QueryEstimate_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const log_gpis::srv::QueryEstimate_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<log_gpis::srv::QueryEstimate_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<log_gpis::srv::QueryEstimate_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      log_gpis::srv::QueryEstimate_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<log_gpis::srv::QueryEstimate_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      log_gpis::srv::QueryEstimate_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<log_gpis::srv::QueryEstimate_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<log_gpis::srv::QueryEstimate_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<log_gpis::srv::QueryEstimate_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__log_gpis__srv__QueryEstimate_Response
    std::shared_ptr<log_gpis::srv::QueryEstimate_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__log_gpis__srv__QueryEstimate_Response
    std::shared_ptr<log_gpis::srv::QueryEstimate_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QueryEstimate_Response_ & other) const
  {
    if (this->estimate != other.estimate) {
      return false;
    }
    if (this->gradient_estimate != other.gradient_estimate) {
      return false;
    }
    if (this->hessian_estimate != other.hessian_estimate) {
      return false;
    }
    return true;
  }
  bool operator!=(const QueryEstimate_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QueryEstimate_Response_

// alias to use template instance with default allocator
using QueryEstimate_Response =
  log_gpis::srv::QueryEstimate_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace log_gpis

namespace log_gpis
{

namespace srv
{

struct QueryEstimate
{
  using Request = log_gpis::srv::QueryEstimate_Request;
  using Response = log_gpis::srv::QueryEstimate_Response;
};

}  // namespace srv

}  // namespace log_gpis

#endif  // LOG_GPIS__SRV__DETAIL__QUERY_ESTIMATE__STRUCT_HPP_
