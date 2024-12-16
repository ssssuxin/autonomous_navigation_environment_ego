# generated from rosidl_generator_py/resource/_idl.py.em
# with input from traj_utils:msg/MultiBsplines.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MultiBsplines(type):
    """Metaclass of message 'MultiBsplines'."""

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
                'traj_utils.msg.MultiBsplines')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__multi_bsplines
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__multi_bsplines
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__multi_bsplines
            cls._TYPE_SUPPORT = module.type_support_msg__msg__multi_bsplines
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__multi_bsplines

            from traj_utils.msg import Bspline
            if Bspline.__class__._TYPE_SUPPORT is None:
                Bspline.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MultiBsplines(metaclass=Metaclass_MultiBsplines):
    """Message class 'MultiBsplines'."""

    __slots__ = [
        '_drone_id_from',
        '_traj',
    ]

    _fields_and_field_types = {
        'drone_id_from': 'int32',
        'traj': 'sequence<traj_utils/Bspline>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['traj_utils', 'msg'], 'Bspline')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.drone_id_from = kwargs.get('drone_id_from', int())
        self.traj = kwargs.get('traj', [])

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
        if self.drone_id_from != other.drone_id_from:
            return False
        if self.traj != other.traj:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def drone_id_from(self):
        """Message field 'drone_id_from'."""
        return self._drone_id_from

    @drone_id_from.setter
    def drone_id_from(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'drone_id_from' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'drone_id_from' field must be an integer in [-2147483648, 2147483647]"
        self._drone_id_from = value

    @property
    def traj(self):
        """Message field 'traj'."""
        return self._traj

    @traj.setter
    def traj(self, value):
        if __debug__:
            from traj_utils.msg import Bspline
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
                 all(isinstance(v, Bspline) for v in value) and
                 True), \
                "The 'traj' field must be a set or sequence and each value of type 'Bspline'"
        self._traj = value
