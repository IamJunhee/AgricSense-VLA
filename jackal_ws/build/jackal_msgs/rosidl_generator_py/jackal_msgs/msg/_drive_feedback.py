# generated from rosidl_generator_py/resource/_idl.py.em
# with input from jackal_msgs:msg/DriveFeedback.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DriveFeedback(type):
    """Metaclass of message 'DriveFeedback'."""

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
                'jackal_msgs.msg.DriveFeedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__drive_feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__drive_feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__drive_feedback
            cls._TYPE_SUPPORT = module.type_support_msg__msg__drive_feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__drive_feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DriveFeedback(metaclass=Metaclass_DriveFeedback):
    """Message class 'DriveFeedback'."""

    __slots__ = [
        '_current',
        '_duty_cycle',
        '_bridge_temperature',
        '_motor_temperature',
        '_measured_velocity',
        '_measured_travel',
        '_driver_fault',
    ]

    _fields_and_field_types = {
        'current': 'float',
        'duty_cycle': 'float',
        'bridge_temperature': 'float',
        'motor_temperature': 'float',
        'measured_velocity': 'float',
        'measured_travel': 'float',
        'driver_fault': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.current = kwargs.get('current', float())
        self.duty_cycle = kwargs.get('duty_cycle', float())
        self.bridge_temperature = kwargs.get('bridge_temperature', float())
        self.motor_temperature = kwargs.get('motor_temperature', float())
        self.measured_velocity = kwargs.get('measured_velocity', float())
        self.measured_travel = kwargs.get('measured_travel', float())
        self.driver_fault = kwargs.get('driver_fault', bool())

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
        if self.current != other.current:
            return False
        if self.duty_cycle != other.duty_cycle:
            return False
        if self.bridge_temperature != other.bridge_temperature:
            return False
        if self.motor_temperature != other.motor_temperature:
            return False
        if self.measured_velocity != other.measured_velocity:
            return False
        if self.measured_travel != other.measured_travel:
            return False
        if self.driver_fault != other.driver_fault:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current(self):
        """Message field 'current'."""
        return self._current

    @current.setter
    def current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._current = value

    @builtins.property
    def duty_cycle(self):
        """Message field 'duty_cycle'."""
        return self._duty_cycle

    @duty_cycle.setter
    def duty_cycle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'duty_cycle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'duty_cycle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._duty_cycle = value

    @builtins.property
    def bridge_temperature(self):
        """Message field 'bridge_temperature'."""
        return self._bridge_temperature

    @bridge_temperature.setter
    def bridge_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bridge_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bridge_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bridge_temperature = value

    @builtins.property
    def motor_temperature(self):
        """Message field 'motor_temperature'."""
        return self._motor_temperature

    @motor_temperature.setter
    def motor_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'motor_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._motor_temperature = value

    @builtins.property
    def measured_velocity(self):
        """Message field 'measured_velocity'."""
        return self._measured_velocity

    @measured_velocity.setter
    def measured_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'measured_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'measured_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._measured_velocity = value

    @builtins.property
    def measured_travel(self):
        """Message field 'measured_travel'."""
        return self._measured_travel

    @measured_travel.setter
    def measured_travel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'measured_travel' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'measured_travel' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._measured_travel = value

    @builtins.property
    def driver_fault(self):
        """Message field 'driver_fault'."""
        return self._driver_fault

    @driver_fault.setter
    def driver_fault(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'driver_fault' field must be of type 'bool'"
        self._driver_fault = value
