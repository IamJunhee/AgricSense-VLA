# generated from rosidl_generator_py/resource/_idl.py.em
# with input from jackal_msgs:msg/Feedback.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Feedback(type):
    """Metaclass of message 'Feedback'."""

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
            module = import_type_support('jackal_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'jackal_msgs.msg.Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__msg__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__feedback

            from jackal_msgs.msg import DriveFeedback
            if DriveFeedback.__class__._TYPE_SUPPORT is None:
                DriveFeedback.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Feedback(metaclass=Metaclass_Feedback):
    """Message class 'Feedback'."""

    __slots__ = [
        '_header',
        '_drivers',
        '_commanded_mode',
        '_actual_mode',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'drivers': 'jackal_msgs/DriveFeedback[2]',
        'commanded_mode': 'int8',
        'actual_mode': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['jackal_msgs', 'msg'], 'DriveFeedback'), 2),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from jackal_msgs.msg import DriveFeedback
        self.drivers = kwargs.get(
            'drivers',
            [DriveFeedback() for x in range(2)]
        )
        self.commanded_mode = kwargs.get('commanded_mode', int())
        self.actual_mode = kwargs.get('actual_mode', int())

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
        if self.header != other.header:
            return False
        if self.drivers != other.drivers:
            return False
        if self.commanded_mode != other.commanded_mode:
            return False
        if self.actual_mode != other.actual_mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def drivers(self):
        """Message field 'drivers'."""
        return self._drivers

    @drivers.setter
    def drivers(self, value):
        if __debug__:
            from jackal_msgs.msg import DriveFeedback
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
                 all(isinstance(v, DriveFeedback) for v in value) and
                 True), \
                "The 'drivers' field must be a set or sequence with length 2 and each value of type 'DriveFeedback'"
        self._drivers = value

    @builtins.property
    def commanded_mode(self):
        """Message field 'commanded_mode'."""
        return self._commanded_mode

    @commanded_mode.setter
    def commanded_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'commanded_mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'commanded_mode' field must be an integer in [-128, 127]"
        self._commanded_mode = value

    @builtins.property
    def actual_mode(self):
        """Message field 'actual_mode'."""
        return self._actual_mode

    @actual_mode.setter
    def actual_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'actual_mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'actual_mode' field must be an integer in [-128, 127]"
        self._actual_mode = value
