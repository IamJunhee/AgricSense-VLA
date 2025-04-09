# generated from rosidl_generator_py/resource/_idl.py.em
# with input from jackal_msgs:msg/Status.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Status(type):
    """Metaclass of message 'Status'."""

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
                'jackal_msgs.msg.Status')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__status

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

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


class Status(metaclass=Metaclass_Status):
    """Message class 'Status'."""

    __slots__ = [
        '_header',
        '_hardware_id',
        '_firmware_commit',
        '_mcu_uptime',
        '_connection_uptime',
        '_pcb_temperature',
        '_mcu_temperature',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'hardware_id': 'string',
        'firmware_commit': 'string',
        'mcu_uptime': 'builtin_interfaces/Duration',
        'connection_uptime': 'builtin_interfaces/Duration',
        'pcb_temperature': 'float',
        'mcu_temperature': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.hardware_id = kwargs.get('hardware_id', str())
        self.firmware_commit = kwargs.get('firmware_commit', str())
        from builtin_interfaces.msg import Duration
        self.mcu_uptime = kwargs.get('mcu_uptime', Duration())
        from builtin_interfaces.msg import Duration
        self.connection_uptime = kwargs.get('connection_uptime', Duration())
        self.pcb_temperature = kwargs.get('pcb_temperature', float())
        self.mcu_temperature = kwargs.get('mcu_temperature', float())

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
        if self.hardware_id != other.hardware_id:
            return False
        if self.firmware_commit != other.firmware_commit:
            return False
        if self.mcu_uptime != other.mcu_uptime:
            return False
        if self.connection_uptime != other.connection_uptime:
            return False
        if self.pcb_temperature != other.pcb_temperature:
            return False
        if self.mcu_temperature != other.mcu_temperature:
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
    def hardware_id(self):
        """Message field 'hardware_id'."""
        return self._hardware_id

    @hardware_id.setter
    def hardware_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'hardware_id' field must be of type 'str'"
        self._hardware_id = value

    @builtins.property
    def firmware_commit(self):
        """Message field 'firmware_commit'."""
        return self._firmware_commit

    @firmware_commit.setter
    def firmware_commit(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'firmware_commit' field must be of type 'str'"
        self._firmware_commit = value

    @builtins.property
    def mcu_uptime(self):
        """Message field 'mcu_uptime'."""
        return self._mcu_uptime

    @mcu_uptime.setter
    def mcu_uptime(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'mcu_uptime' field must be a sub message of type 'Duration'"
        self._mcu_uptime = value

    @builtins.property
    def connection_uptime(self):
        """Message field 'connection_uptime'."""
        return self._connection_uptime

    @connection_uptime.setter
    def connection_uptime(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'connection_uptime' field must be a sub message of type 'Duration'"
        self._connection_uptime = value

    @builtins.property
    def pcb_temperature(self):
        """Message field 'pcb_temperature'."""
        return self._pcb_temperature

    @pcb_temperature.setter
    def pcb_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pcb_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pcb_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pcb_temperature = value

    @builtins.property
    def mcu_temperature(self):
        """Message field 'mcu_temperature'."""
        return self._mcu_temperature

    @mcu_temperature.setter
    def mcu_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'mcu_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'mcu_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._mcu_temperature = value
