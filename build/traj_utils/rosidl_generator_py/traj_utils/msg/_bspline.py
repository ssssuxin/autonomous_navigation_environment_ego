# generated from rosidl_generator_py/resource/_idl.py.em
# with input from traj_utils:msg/Bspline.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'knots'
# Member 'yaw_pts'
import array  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Bspline(type):
    """Metaclass of message 'Bspline'."""

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
            module = import_type_support('traj_utils')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'traj_utils.msg.Bspline')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bspline
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bspline
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bspline
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bspline
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bspline

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Bspline(metaclass=Metaclass_Bspline):
    """Message class 'Bspline'."""

    __slots__ = [
        '_drone_id',
        '_order',
        '_traj_id',
        '_start_time',
        '_knots',
        '_pos_pts',
        '_yaw_pts',
        '_yaw_dt',
    ]

    _fields_and_field_types = {
        'drone_id': 'int32',
        'order': 'int32',
        'traj_id': 'int64',
        'start_time': 'builtin_interfaces/Time',
        'knots': 'sequence<double>',
        'pos_pts': 'sequence<geometry_msgs/Point>',
        'yaw_pts': 'sequence<double>',
        'yaw_dt': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.drone_id = kwargs.get('drone_id', int())
        self.order = kwargs.get('order', int())
        self.traj_id = kwargs.get('traj_id', int())
        from builtin_interfaces.msg import Time
        self.start_time = kwargs.get('start_time', Time())
        self.knots = array.array('d', kwargs.get('knots', []))
        self.pos_pts = kwargs.get('pos_pts', [])
        self.yaw_pts = array.array('d', kwargs.get('yaw_pts', []))
        self.yaw_dt = kwargs.get('yaw_dt', float())

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
        if self.drone_id != other.drone_id:
            return False
        if self.order != other.order:
            return False
        if self.traj_id != other.traj_id:
            return False
        if self.start_time != other.start_time:
            return False
        if self.knots != other.knots:
            return False
        if self.pos_pts != other.pos_pts:
            return False
        if self.yaw_pts != other.yaw_pts:
            return False
        if self.yaw_dt != other.yaw_dt:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def drone_id(self):
        """Message field 'drone_id'."""
        return self._drone_id

    @drone_id.setter
    def drone_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'drone_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'drone_id' field must be an integer in [-2147483648, 2147483647]"
        self._drone_id = value

    @property
    def order(self):
        """Message field 'order'."""
        return self._order

    @order.setter
    def order(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'order' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'order' field must be an integer in [-2147483648, 2147483647]"
        self._order = value

    @property
    def traj_id(self):
        """Message field 'traj_id'."""
        return self._traj_id

    @traj_id.setter
    def traj_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'traj_id' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'traj_id' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._traj_id = value

    @property
    def start_time(self):
        """Message field 'start_time'."""
        return self._start_time

    @start_time.setter
    def start_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'start_time' field must be a sub message of type 'Time'"
        self._start_time = value

    @property
    def knots(self):
        """Message field 'knots'."""
        return self._knots

    @knots.setter
    def knots(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'knots' array.array() must have the type code of 'd'"
            self._knots = value
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
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'knots' field must be a set or sequence and each value of type 'float'"
        self._knots = array.array('d', value)

    @property
    def pos_pts(self):
        """Message field 'pos_pts'."""
        return self._pos_pts

    @pos_pts.setter
    def pos_pts(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
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
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'pos_pts' field must be a set or sequence and each value of type 'Point'"
        self._pos_pts = value

    @property
    def yaw_pts(self):
        """Message field 'yaw_pts'."""
        return self._yaw_pts

    @yaw_pts.setter
    def yaw_pts(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'yaw_pts' array.array() must have the type code of 'd'"
            self._yaw_pts = value
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
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'yaw_pts' field must be a set or sequence and each value of type 'float'"
        self._yaw_pts = array.array('d', value)

    @property
    def yaw_dt(self):
        """Message field 'yaw_dt'."""
        return self._yaw_dt

    @yaw_dt.setter
    def yaw_dt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_dt' field must be of type 'float'"
        self._yaw_dt = value
