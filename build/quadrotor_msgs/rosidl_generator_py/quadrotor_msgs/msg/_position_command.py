# generated from rosidl_generator_py/resource/_idl.py.em
# with input from quadrotor_msgs:msg/PositionCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'kx'
# Member 'kv'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PositionCommand(type):
    """Metaclass of message 'PositionCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'TRAJECTORY_STATUS_EMPTY': 0,
        'TRAJECTORY_STATUS_READY': 1,
        'TRAJECTORY_STATUS_COMPLETED': 3,
        'TRAJECTROY_STATUS_ABORT': 4,
        'TRAJECTORY_STATUS_ILLEGAL_START': 5,
        'TRAJECTORY_STATUS_ILLEGAL_FINAL': 6,
        'TRAJECTORY_STATUS_IMPOSSIBLE': 7,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('quadrotor_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'quadrotor_msgs.msg.PositionCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__position_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__position_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__position_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__position_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__position_command

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'TRAJECTORY_STATUS_EMPTY': cls.__constants['TRAJECTORY_STATUS_EMPTY'],
            'TRAJECTORY_STATUS_READY': cls.__constants['TRAJECTORY_STATUS_READY'],
            'TRAJECTORY_STATUS_COMPLETED': cls.__constants['TRAJECTORY_STATUS_COMPLETED'],
            'TRAJECTROY_STATUS_ABORT': cls.__constants['TRAJECTROY_STATUS_ABORT'],
            'TRAJECTORY_STATUS_ILLEGAL_START': cls.__constants['TRAJECTORY_STATUS_ILLEGAL_START'],
            'TRAJECTORY_STATUS_ILLEGAL_FINAL': cls.__constants['TRAJECTORY_STATUS_ILLEGAL_FINAL'],
            'TRAJECTORY_STATUS_IMPOSSIBLE': cls.__constants['TRAJECTORY_STATUS_IMPOSSIBLE'],
        }

    @property
    def TRAJECTORY_STATUS_EMPTY(self):
        """Message constant 'TRAJECTORY_STATUS_EMPTY'."""
        return Metaclass_PositionCommand.__constants['TRAJECTORY_STATUS_EMPTY']

    @property
    def TRAJECTORY_STATUS_READY(self):
        """Message constant 'TRAJECTORY_STATUS_READY'."""
        return Metaclass_PositionCommand.__constants['TRAJECTORY_STATUS_READY']

    @property
    def TRAJECTORY_STATUS_COMPLETED(self):
        """Message constant 'TRAJECTORY_STATUS_COMPLETED'."""
        return Metaclass_PositionCommand.__constants['TRAJECTORY_STATUS_COMPLETED']

    @property
    def TRAJECTROY_STATUS_ABORT(self):
        """Message constant 'TRAJECTROY_STATUS_ABORT'."""
        return Metaclass_PositionCommand.__constants['TRAJECTROY_STATUS_ABORT']

    @property
    def TRAJECTORY_STATUS_ILLEGAL_START(self):
        """Message constant 'TRAJECTORY_STATUS_ILLEGAL_START'."""
        return Metaclass_PositionCommand.__constants['TRAJECTORY_STATUS_ILLEGAL_START']

    @property
    def TRAJECTORY_STATUS_ILLEGAL_FINAL(self):
        """Message constant 'TRAJECTORY_STATUS_ILLEGAL_FINAL'."""
        return Metaclass_PositionCommand.__constants['TRAJECTORY_STATUS_ILLEGAL_FINAL']

    @property
    def TRAJECTORY_STATUS_IMPOSSIBLE(self):
        """Message constant 'TRAJECTORY_STATUS_IMPOSSIBLE'."""
        return Metaclass_PositionCommand.__constants['TRAJECTORY_STATUS_IMPOSSIBLE']


class PositionCommand(metaclass=Metaclass_PositionCommand):
    """
    Message class 'PositionCommand'.

    Constants:
      TRAJECTORY_STATUS_EMPTY
      TRAJECTORY_STATUS_READY
      TRAJECTORY_STATUS_COMPLETED
      TRAJECTROY_STATUS_ABORT
      TRAJECTORY_STATUS_ILLEGAL_START
      TRAJECTORY_STATUS_ILLEGAL_FINAL
      TRAJECTORY_STATUS_IMPOSSIBLE
    """

    __slots__ = [
        '_header',
        '_position',
        '_velocity',
        '_acceleration',
        '_yaw',
        '_yaw_dot',
        '_kx',
        '_kv',
        '_trajectory_id',
        '_trajectory_flag',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'position': 'geometry_msgs/Point',
        'velocity': 'geometry_msgs/Vector3',
        'acceleration': 'geometry_msgs/Vector3',
        'yaw': 'double',
        'yaw_dot': 'double',
        'kx': 'double[3]',
        'kv': 'double[3]',
        'trajectory_id': 'uint32',
        'trajectory_flag': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from geometry_msgs.msg import Point
        self.position = kwargs.get('position', Point())
        from geometry_msgs.msg import Vector3
        self.velocity = kwargs.get('velocity', Vector3())
        from geometry_msgs.msg import Vector3
        self.acceleration = kwargs.get('acceleration', Vector3())
        self.yaw = kwargs.get('yaw', float())
        self.yaw_dot = kwargs.get('yaw_dot', float())
        if 'kx' not in kwargs:
            self.kx = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.kx = numpy.array(kwargs.get('kx'), dtype=numpy.float64)
            assert self.kx.shape == (3, )
        if 'kv' not in kwargs:
            self.kv = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.kv = numpy.array(kwargs.get('kv'), dtype=numpy.float64)
            assert self.kv.shape == (3, )
        self.trajectory_id = kwargs.get('trajectory_id', int())
        self.trajectory_flag = kwargs.get('trajectory_flag', int())

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
        if self.position != other.position:
            return False
        if self.velocity != other.velocity:
            return False
        if self.acceleration != other.acceleration:
            return False
        if self.yaw != other.yaw:
            return False
        if self.yaw_dot != other.yaw_dot:
            return False
        if all(self.kx != other.kx):
            return False
        if all(self.kv != other.kv):
            return False
        if self.trajectory_id != other.trajectory_id:
            return False
        if self.trajectory_flag != other.trajectory_flag:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
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

    @property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'position' field must be a sub message of type 'Point'"
        self._position = value

    @property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'velocity' field must be a sub message of type 'Vector3'"
        self._velocity = value

    @property
    def acceleration(self):
        """Message field 'acceleration'."""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'acceleration' field must be a sub message of type 'Vector3'"
        self._acceleration = value

    @property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
        self._yaw = value

    @property
    def yaw_dot(self):
        """Message field 'yaw_dot'."""
        return self._yaw_dot

    @yaw_dot.setter
    def yaw_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_dot' field must be of type 'float'"
        self._yaw_dot = value

    @property
    def kx(self):
        """Message field 'kx'."""
        return self._kx

    @kx.setter
    def kx(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'kx' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'kx' numpy.ndarray() must have a size of 3"
            self._kx = value
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
                 True), \
                "The 'kx' field must be a set or sequence with length 3 and each value of type 'float'"
        self._kx = numpy.array(value, dtype=numpy.float64)

    @property
    def kv(self):
        """Message field 'kv'."""
        return self._kv

    @kv.setter
    def kv(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'kv' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'kv' numpy.ndarray() must have a size of 3"
            self._kv = value
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
                 True), \
                "The 'kv' field must be a set or sequence with length 3 and each value of type 'float'"
        self._kv = numpy.array(value, dtype=numpy.float64)

    @property
    def trajectory_id(self):
        """Message field 'trajectory_id'."""
        return self._trajectory_id

    @trajectory_id.setter
    def trajectory_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'trajectory_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'trajectory_id' field must be an unsigned integer in [0, 4294967295]"
        self._trajectory_id = value

    @property
    def trajectory_flag(self):
        """Message field 'trajectory_flag'."""
        return self._trajectory_flag

    @trajectory_flag.setter
    def trajectory_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'trajectory_flag' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'trajectory_flag' field must be an unsigned integer in [0, 255]"
        self._trajectory_flag = value
