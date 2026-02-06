# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rosbasic_move_msgs:srv/Control.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Control_Request(type):
    """Metaclass of message 'Control_Request'."""

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
            module = import_type_support('rosbasic_move_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rosbasic_move_msgs.srv.Control_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__control__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__control__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__control__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__control__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__control__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Control_Request(metaclass=Metaclass_Control_Request):
    """Message class 'Control_Request'."""

    __slots__ = [
        '_quangduong',
        '_khoangcach',
    ]

    _fields_and_field_types = {
        'quangduong': 'float',
        'khoangcach': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.quangduong = kwargs.get('quangduong', float())
        self.khoangcach = kwargs.get('khoangcach', float())

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
        if self.quangduong != other.quangduong:
            return False
        if self.khoangcach != other.khoangcach:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def quangduong(self):
        """Message field 'quangduong'."""
        return self._quangduong

    @quangduong.setter
    def quangduong(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'quangduong' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'quangduong' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._quangduong = value

    @builtins.property
    def khoangcach(self):
        """Message field 'khoangcach'."""
        return self._khoangcach

    @khoangcach.setter
    def khoangcach(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'khoangcach' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'khoangcach' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._khoangcach = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Control_Response(type):
    """Metaclass of message 'Control_Response'."""

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
            module = import_type_support('rosbasic_move_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rosbasic_move_msgs.srv.Control_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__control__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__control__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__control__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__control__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__control__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Control_Response(metaclass=Metaclass_Control_Response):
    """Message class 'Control_Response'."""

    __slots__ = [
        '_success',
        '_notification',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'notification': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.notification = kwargs.get('notification', str())

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
        if self.success != other.success:
            return False
        if self.notification != other.notification:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def notification(self):
        """Message field 'notification'."""
        return self._notification

    @notification.setter
    def notification(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'notification' field must be of type 'str'"
        self._notification = value


class Metaclass_Control(type):
    """Metaclass of service 'Control'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rosbasic_move_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rosbasic_move_msgs.srv.Control')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__control

            from rosbasic_move_msgs.srv import _control
            if _control.Metaclass_Control_Request._TYPE_SUPPORT is None:
                _control.Metaclass_Control_Request.__import_type_support__()
            if _control.Metaclass_Control_Response._TYPE_SUPPORT is None:
                _control.Metaclass_Control_Response.__import_type_support__()


class Control(metaclass=Metaclass_Control):
    from rosbasic_move_msgs.srv._control import Control_Request as Request
    from rosbasic_move_msgs.srv._control import Control_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
