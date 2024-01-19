# generated from rosidl_generator_py/resource/_idl.py.em
# with input from log_gpis:srv/QueryEstimate.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_QueryEstimate_Request(type):
    """Metaclass of message 'QueryEstimate_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('log_gpis')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'log_gpis.srv.QueryEstimate_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__query_estimate__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__query_estimate__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__query_estimate__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__query_estimate__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__query_estimate__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class QueryEstimate_Request(metaclass=Metaclass_QueryEstimate_Request):
    """Message class 'QueryEstimate_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'gradient_estimate'
# Member 'hessian_estimate'
import numpy  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_QueryEstimate_Response(type):
    """Metaclass of message 'QueryEstimate_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('log_gpis')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'log_gpis.srv.QueryEstimate_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__query_estimate__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__query_estimate__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__query_estimate__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__query_estimate__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__query_estimate__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class QueryEstimate_Response(metaclass=Metaclass_QueryEstimate_Response):
    """Message class 'QueryEstimate_Response'."""

    __slots__ = [
        '_estimate',
        '_gradient_estimate',
        '_hessian_estimate',
    ]

    _fields_and_field_types = {
        'estimate': 'double',
        'gradient_estimate': 'double[3]',
        'hessian_estimate': 'double[9]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 9),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.estimate = kwargs.get('estimate', float())
        if 'gradient_estimate' not in kwargs:
            self.gradient_estimate = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.gradient_estimate = numpy.array(kwargs.get('gradient_estimate'), dtype=numpy.float64)
            assert self.gradient_estimate.shape == (3, )
        if 'hessian_estimate' not in kwargs:
            self.hessian_estimate = numpy.zeros(9, dtype=numpy.float64)
        else:
            self.hessian_estimate = numpy.array(kwargs.get('hessian_estimate'), dtype=numpy.float64)
            assert self.hessian_estimate.shape == (9, )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.estimate != other.estimate:
            return False
        if all(self.gradient_estimate != other.gradient_estimate):
            return False
        if all(self.hessian_estimate != other.hessian_estimate):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def estimate(self):
        """Message field 'estimate'."""
        return self._estimate

    @estimate.setter
    def estimate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'estimate' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'estimate' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._estimate = value

    @builtins.property
    def gradient_estimate(self):
        """Message field 'gradient_estimate'."""
        return self._gradient_estimate

    @gradient_estimate.setter
    def gradient_estimate(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'gradient_estimate' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'gradient_estimate' numpy.ndarray() must have a size of 3"
            self._gradient_estimate = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'gradient_estimate' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._gradient_estimate = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def hessian_estimate(self):
        """Message field 'hessian_estimate'."""
        return self._hessian_estimate

    @hessian_estimate.setter
    def hessian_estimate(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'hessian_estimate' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 9, \
                "The 'hessian_estimate' numpy.ndarray() must have a size of 9"
            self._hessian_estimate = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 9 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'hessian_estimate' field must be a set or sequence with length 9 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._hessian_estimate = numpy.array(value, dtype=numpy.float64)


class Metaclass_QueryEstimate(type):
    """Metaclass of service 'QueryEstimate'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('log_gpis')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'log_gpis.srv.QueryEstimate')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__query_estimate

            from log_gpis.srv import _query_estimate
            if _query_estimate.Metaclass_QueryEstimate_Request._TYPE_SUPPORT is None:
                _query_estimate.Metaclass_QueryEstimate_Request.__import_type_support__()
            if _query_estimate.Metaclass_QueryEstimate_Response._TYPE_SUPPORT is None:
                _query_estimate.Metaclass_QueryEstimate_Response.__import_type_support__()


class QueryEstimate(metaclass=Metaclass_QueryEstimate):
    from log_gpis.srv._query_estimate import QueryEstimate_Request as Request
    from log_gpis.srv._query_estimate import QueryEstimate_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
