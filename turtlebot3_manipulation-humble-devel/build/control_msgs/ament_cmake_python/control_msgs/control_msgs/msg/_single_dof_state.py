# generated from rosidl_generator_py/resource/_idl.py.em
# with input from control_msgs:msg/SingleDOFState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SingleDOFState(type):
    """Metaclass of message 'SingleDOFState'."""

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
            module = import_type_support('control_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'control_msgs.msg.SingleDOFState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__single_dof_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__single_dof_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__single_dof_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__single_dof_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__single_dof_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SingleDOFState(metaclass=Metaclass_SingleDOFState):
    """Message class 'SingleDOFState'."""

    __slots__ = [
        '_name',
        '_reference',
        '_feedback',
        '_feedback_dot',
        '_error',
        '_error_dot',
        '_time_step',
        '_output',
    ]

    _fields_and_field_types = {
        'name': 'string',
        'reference': 'double',
        'feedback': 'double',
        'feedback_dot': 'double',
        'error': 'double',
        'error_dot': 'double',
        'time_step': 'double',
        'output': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.name = kwargs.get('name', str())
        self.reference = kwargs.get('reference', float())
        self.feedback = kwargs.get('feedback', float())
        self.feedback_dot = kwargs.get('feedback_dot', float())
        self.error = kwargs.get('error', float())
        self.error_dot = kwargs.get('error_dot', float())
        self.time_step = kwargs.get('time_step', float())
        self.output = kwargs.get('output', float())

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
        if self.name != other.name:
            return False
        if self.reference != other.reference:
            return False
        if self.feedback != other.feedback:
            return False
        if self.feedback_dot != other.feedback_dot:
            return False
        if self.error != other.error:
            return False
        if self.error_dot != other.error_dot:
            return False
        if self.time_step != other.time_step:
            return False
        if self.output != other.output:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property
    def reference(self):
        """Message field 'reference'."""
        return self._reference

    @reference.setter
    def reference(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'reference' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'reference' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._reference = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'feedback' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'feedback' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._feedback = value

    @builtins.property
    def feedback_dot(self):
        """Message field 'feedback_dot'."""
        return self._feedback_dot

    @feedback_dot.setter
    def feedback_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'feedback_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'feedback_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._feedback_dot = value

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'error' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'error' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._error = value

    @builtins.property
    def error_dot(self):
        """Message field 'error_dot'."""
        return self._error_dot

    @error_dot.setter
    def error_dot(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'error_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'error_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._error_dot = value

    @builtins.property
    def time_step(self):
        """Message field 'time_step'."""
        return self._time_step

    @time_step.setter
    def time_step(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'time_step' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'time_step' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._time_step = value

    @builtins.property
    def output(self):
        """Message field 'output'."""
        return self._output

    @output.setter
    def output(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'output' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'output' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._output = value
