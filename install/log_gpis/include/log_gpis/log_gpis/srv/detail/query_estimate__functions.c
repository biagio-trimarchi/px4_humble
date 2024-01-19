// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from log_gpis:srv/QueryEstimate.idl
// generated code does not contain a copyright notice
#include "log_gpis/srv/detail/query_estimate__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
log_gpis__srv__QueryEstimate_Request__init(log_gpis__srv__QueryEstimate_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
log_gpis__srv__QueryEstimate_Request__fini(log_gpis__srv__QueryEstimate_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
log_gpis__srv__QueryEstimate_Request__are_equal(const log_gpis__srv__QueryEstimate_Request * lhs, const log_gpis__srv__QueryEstimate_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
log_gpis__srv__QueryEstimate_Request__copy(
  const log_gpis__srv__QueryEstimate_Request * input,
  log_gpis__srv__QueryEstimate_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

log_gpis__srv__QueryEstimate_Request *
log_gpis__srv__QueryEstimate_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  log_gpis__srv__QueryEstimate_Request * msg = (log_gpis__srv__QueryEstimate_Request *)allocator.allocate(sizeof(log_gpis__srv__QueryEstimate_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(log_gpis__srv__QueryEstimate_Request));
  bool success = log_gpis__srv__QueryEstimate_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
log_gpis__srv__QueryEstimate_Request__destroy(log_gpis__srv__QueryEstimate_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    log_gpis__srv__QueryEstimate_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
log_gpis__srv__QueryEstimate_Request__Sequence__init(log_gpis__srv__QueryEstimate_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  log_gpis__srv__QueryEstimate_Request * data = NULL;

  if (size) {
    data = (log_gpis__srv__QueryEstimate_Request *)allocator.zero_allocate(size, sizeof(log_gpis__srv__QueryEstimate_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = log_gpis__srv__QueryEstimate_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        log_gpis__srv__QueryEstimate_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
log_gpis__srv__QueryEstimate_Request__Sequence__fini(log_gpis__srv__QueryEstimate_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      log_gpis__srv__QueryEstimate_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

log_gpis__srv__QueryEstimate_Request__Sequence *
log_gpis__srv__QueryEstimate_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  log_gpis__srv__QueryEstimate_Request__Sequence * array = (log_gpis__srv__QueryEstimate_Request__Sequence *)allocator.allocate(sizeof(log_gpis__srv__QueryEstimate_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = log_gpis__srv__QueryEstimate_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
log_gpis__srv__QueryEstimate_Request__Sequence__destroy(log_gpis__srv__QueryEstimate_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    log_gpis__srv__QueryEstimate_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
log_gpis__srv__QueryEstimate_Request__Sequence__are_equal(const log_gpis__srv__QueryEstimate_Request__Sequence * lhs, const log_gpis__srv__QueryEstimate_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!log_gpis__srv__QueryEstimate_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
log_gpis__srv__QueryEstimate_Request__Sequence__copy(
  const log_gpis__srv__QueryEstimate_Request__Sequence * input,
  log_gpis__srv__QueryEstimate_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(log_gpis__srv__QueryEstimate_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    log_gpis__srv__QueryEstimate_Request * data =
      (log_gpis__srv__QueryEstimate_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!log_gpis__srv__QueryEstimate_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          log_gpis__srv__QueryEstimate_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!log_gpis__srv__QueryEstimate_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
log_gpis__srv__QueryEstimate_Response__init(log_gpis__srv__QueryEstimate_Response * msg)
{
  if (!msg) {
    return false;
  }
  // estimate
  // gradient_estimate
  // hessian_estimate
  return true;
}

void
log_gpis__srv__QueryEstimate_Response__fini(log_gpis__srv__QueryEstimate_Response * msg)
{
  if (!msg) {
    return;
  }
  // estimate
  // gradient_estimate
  // hessian_estimate
}

bool
log_gpis__srv__QueryEstimate_Response__are_equal(const log_gpis__srv__QueryEstimate_Response * lhs, const log_gpis__srv__QueryEstimate_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // estimate
  if (lhs->estimate != rhs->estimate) {
    return false;
  }
  // gradient_estimate
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gradient_estimate[i] != rhs->gradient_estimate[i]) {
      return false;
    }
  }
  // hessian_estimate
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->hessian_estimate[i] != rhs->hessian_estimate[i]) {
      return false;
    }
  }
  return true;
}

bool
log_gpis__srv__QueryEstimate_Response__copy(
  const log_gpis__srv__QueryEstimate_Response * input,
  log_gpis__srv__QueryEstimate_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // estimate
  output->estimate = input->estimate;
  // gradient_estimate
  for (size_t i = 0; i < 3; ++i) {
    output->gradient_estimate[i] = input->gradient_estimate[i];
  }
  // hessian_estimate
  for (size_t i = 0; i < 9; ++i) {
    output->hessian_estimate[i] = input->hessian_estimate[i];
  }
  return true;
}

log_gpis__srv__QueryEstimate_Response *
log_gpis__srv__QueryEstimate_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  log_gpis__srv__QueryEstimate_Response * msg = (log_gpis__srv__QueryEstimate_Response *)allocator.allocate(sizeof(log_gpis__srv__QueryEstimate_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(log_gpis__srv__QueryEstimate_Response));
  bool success = log_gpis__srv__QueryEstimate_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
log_gpis__srv__QueryEstimate_Response__destroy(log_gpis__srv__QueryEstimate_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    log_gpis__srv__QueryEstimate_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
log_gpis__srv__QueryEstimate_Response__Sequence__init(log_gpis__srv__QueryEstimate_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  log_gpis__srv__QueryEstimate_Response * data = NULL;

  if (size) {
    data = (log_gpis__srv__QueryEstimate_Response *)allocator.zero_allocate(size, sizeof(log_gpis__srv__QueryEstimate_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = log_gpis__srv__QueryEstimate_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        log_gpis__srv__QueryEstimate_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
log_gpis__srv__QueryEstimate_Response__Sequence__fini(log_gpis__srv__QueryEstimate_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      log_gpis__srv__QueryEstimate_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

log_gpis__srv__QueryEstimate_Response__Sequence *
log_gpis__srv__QueryEstimate_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  log_gpis__srv__QueryEstimate_Response__Sequence * array = (log_gpis__srv__QueryEstimate_Response__Sequence *)allocator.allocate(sizeof(log_gpis__srv__QueryEstimate_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = log_gpis__srv__QueryEstimate_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
log_gpis__srv__QueryEstimate_Response__Sequence__destroy(log_gpis__srv__QueryEstimate_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    log_gpis__srv__QueryEstimate_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
log_gpis__srv__QueryEstimate_Response__Sequence__are_equal(const log_gpis__srv__QueryEstimate_Response__Sequence * lhs, const log_gpis__srv__QueryEstimate_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!log_gpis__srv__QueryEstimate_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
log_gpis__srv__QueryEstimate_Response__Sequence__copy(
  const log_gpis__srv__QueryEstimate_Response__Sequence * input,
  log_gpis__srv__QueryEstimate_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(log_gpis__srv__QueryEstimate_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    log_gpis__srv__QueryEstimate_Response * data =
      (log_gpis__srv__QueryEstimate_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!log_gpis__srv__QueryEstimate_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          log_gpis__srv__QueryEstimate_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!log_gpis__srv__QueryEstimate_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
