# generated from rosidl_generator_py/resource/_idl.py.em
# with input from exia_msgs:msg/NavigationGoal.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NavigationGoal(type):
    """Metaclass of message 'NavigationGoal'."""

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
            module = import_type_support('exia_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'exia_msgs.msg.NavigationGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__navigation_goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__navigation_goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__navigation_goal
            cls._TYPE_SUPPORT = module.type_support_msg__msg__navigation_goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__navigation_goal

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigationGoal(metaclass=Metaclass_NavigationGoal):
    """Message class 'NavigationGoal'."""

    __slots__ = [
        '_coord_type',
        '_x',
        '_y',
        '_lat',
        '_lon',
        '_lat_dms',
        '_lon_dms',
        '_origin_lat',
        '_origin_lon',
    ]

    _fields_and_field_types = {
        'coord_type': 'string',
        'x': 'double',
        'y': 'double',
        'lat': 'double',
        'lon': 'double',
        'lat_dms': 'string',
        'lon_dms': 'string',
        'origin_lat': 'double',
        'origin_lon': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.coord_type = kwargs.get('coord_type', str())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.lat = kwargs.get('lat', float())
        self.lon = kwargs.get('lon', float())
        self.lat_dms = kwargs.get('lat_dms', str())
        self.lon_dms = kwargs.get('lon_dms', str())
        self.origin_lat = kwargs.get('origin_lat', float())
        self.origin_lon = kwargs.get('origin_lon', float())

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
        if self.coord_type != other.coord_type:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.lat != other.lat:
            return False
        if self.lon != other.lon:
            return False
        if self.lat_dms != other.lat_dms:
            return False
        if self.lon_dms != other.lon_dms:
            return False
        if self.origin_lat != other.origin_lat:
            return False
        if self.origin_lon != other.origin_lon:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def coord_type(self):
        """Message field 'coord_type'."""
        return self._coord_type

    @coord_type.setter
    def coord_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'coord_type' field must be of type 'str'"
        self._coord_type = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y = value

    @builtins.property
    def lat(self):
        """Message field 'lat'."""
        return self._lat

    @lat.setter
    def lat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lat' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lat' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lat = value

    @builtins.property
    def lon(self):
        """Message field 'lon'."""
        return self._lon

    @lon.setter
    def lon(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lon' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lon' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lon = value

    @builtins.property
    def lat_dms(self):
        """Message field 'lat_dms'."""
        return self._lat_dms

    @lat_dms.setter
    def lat_dms(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'lat_dms' field must be of type 'str'"
        self._lat_dms = value

    @builtins.property
    def lon_dms(self):
        """Message field 'lon_dms'."""
        return self._lon_dms

    @lon_dms.setter
    def lon_dms(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'lon_dms' field must be of type 'str'"
        self._lon_dms = value

    @builtins.property
    def origin_lat(self):
        """Message field 'origin_lat'."""
        return self._origin_lat

    @origin_lat.setter
    def origin_lat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'origin_lat' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'origin_lat' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._origin_lat = value

    @builtins.property
    def origin_lon(self):
        """Message field 'origin_lon'."""
        return self._origin_lon

    @origin_lon.setter
    def origin_lon(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'origin_lon' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'origin_lon' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._origin_lon = value
