# generated from rosidl_generator_py/resource/_idl.py.em
# with input from jackal_msgs:msg/Drive.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'drivers'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Drive(type):
    """Metaclass of message 'Drive'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MODE_VELOCITY': 0,
        'MODE_PWM': 1,
        'MODE_EFFORT': 2,
        'MODE_NONE': -1,
        'LEFT': 0,
        'RIGHT': 1,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('jackal_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'jackal_msgs.msg.Drive')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__drive
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__drive
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__drive
            cls._TYPE_SUPPORT = module.type_support_msg__msg__drive
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__drive

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MODE_VELOCITY': cls.__constants['MODE_VELOCITY'],
            'MODE_PWM': cls.__constants['MODE_PWM'],
            'MODE_EFFORT': cls.__constants['MODE_EFFORT'],
            'MODE_NONE': cls.__constants['MODE_NONE'],
            'LEFT': cls.__constants['LEFT'],
            'RIGHT': cls.__constants['RIGHT'],
        }

    @property
    def MODE_VELOCITY(self):
        """Message constant 'MODE_VELOCITY'."""
        return Metaclass_Drive.__constants['MODE_VELOCITY']

    @property
    def MODE_PWM(self):
        """Message constant 'MODE_PWM'."""
        return Metaclass_Drive.__constants['MODE_PWM']

    @property
    def MODE_EFFORT(self):
        """Message constant 'MODE_EFFORT'."""
        return Metaclass_Drive.__constants['MODE_EFFORT']

    @property
    def MODE_NONE(self):
        """Message constant 'MODE_NONE'."""
        return Metaclass_Drive.__constants['MODE_NONE']

    @property
    def LEFT(self):
        """Message constant 'LEFT'."""
        return Metaclass_Drive.__constants['LEFT']

    @property
    def RIGHT(self):
        """Message constant 'RIGHT'."""
        return Metaclass_Drive.__constants['RIGHT']


class Drive(metaclass=Metaclass_Drive):
    """
    Message class 'Drive'.

    Constants:
      MODE_VELOCITY
      MODE_PWM
      MODE_EFFORT
      MODE_NONE
      LEFT
      RIGHT
    """

    __slots__ = [
        '_mode',
        '_drivers',
    ]

    _fields_and_field_types = {
        'mode': 'int8',
        'drivers': 'float[2]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 2),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode = kwargs.get('mode', int())
        if 'drivers' not in kwargs:
            self.drivers = numpy.zeros(2, dtype=numpy.float32)
        else:
            self.drivers = numpy.array(kwargs.get('drivers'), dtype=numpy.float32)
            assert self.drivers.shape == (2, )

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
        if self.mode != other.mode:
            return False
        if all(self.drivers != other.drivers):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'mode' field must be an integer in [-128, 127]"
        self._mode = value

    @builtins.property
    def drivers(self):
        """Message field 'drivers'."""
        return self._drivers

    @drivers.setter
    def drivers(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'drivers' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 2, \
                "The 'drivers' numpy.ndarray() must have a size of 2"
            self._drivers = value
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
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'drivers' field must be a set or sequence with length 2 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._drivers = numpy.array(value, dtype=numpy.float32)
