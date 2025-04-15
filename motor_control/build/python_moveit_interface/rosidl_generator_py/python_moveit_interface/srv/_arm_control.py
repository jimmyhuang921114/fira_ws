# generated from rosidl_generator_py/resource/_idl.py.em
# with input from python_moveit_interface:srv/ArmControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ArmControl_Request(type):
    """Metaclass of message 'ArmControl_Request'."""

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
            module = import_type_support('python_moveit_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'python_moveit_interface.srv.ArmControl_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__arm_control__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__arm_control__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__arm_control__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__arm_control__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__arm_control__request

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ArmControl_Request(metaclass=Metaclass_ArmControl_Request):
    """Message class 'ArmControl_Request'."""

    __slots__ = [
        '_task_name',
        '_trigger_grab_flow',
        '_trigger_place_flow',
        '_target_zone',
        '_target_pose',
        '_named_pose',
        '_gripper_close',
    ]

    _fields_and_field_types = {
        'task_name': 'string',
        'trigger_grab_flow': 'boolean',
        'trigger_place_flow': 'boolean',
        'target_zone': 'string',
        'target_pose': 'geometry_msgs/Pose',
        'named_pose': 'string',
        'gripper_close': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.task_name = kwargs.get('task_name', str())
        self.trigger_grab_flow = kwargs.get('trigger_grab_flow', bool())
        self.trigger_place_flow = kwargs.get('trigger_place_flow', bool())
        self.target_zone = kwargs.get('target_zone', str())
        from geometry_msgs.msg import Pose
        self.target_pose = kwargs.get('target_pose', Pose())
        self.named_pose = kwargs.get('named_pose', str())
        self.gripper_close = kwargs.get('gripper_close', bool())

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
        if self.task_name != other.task_name:
            return False
        if self.trigger_grab_flow != other.trigger_grab_flow:
            return False
        if self.trigger_place_flow != other.trigger_place_flow:
            return False
        if self.target_zone != other.target_zone:
            return False
        if self.target_pose != other.target_pose:
            return False
        if self.named_pose != other.named_pose:
            return False
        if self.gripper_close != other.gripper_close:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def task_name(self):
        """Message field 'task_name'."""
        return self._task_name

    @task_name.setter
    def task_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'task_name' field must be of type 'str'"
        self._task_name = value

    @builtins.property
    def trigger_grab_flow(self):
        """Message field 'trigger_grab_flow'."""
        return self._trigger_grab_flow

    @trigger_grab_flow.setter
    def trigger_grab_flow(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'trigger_grab_flow' field must be of type 'bool'"
        self._trigger_grab_flow = value

    @builtins.property
    def trigger_place_flow(self):
        """Message field 'trigger_place_flow'."""
        return self._trigger_place_flow

    @trigger_place_flow.setter
    def trigger_place_flow(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'trigger_place_flow' field must be of type 'bool'"
        self._trigger_place_flow = value

    @builtins.property
    def target_zone(self):
        """Message field 'target_zone'."""
        return self._target_zone

    @target_zone.setter
    def target_zone(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'target_zone' field must be of type 'str'"
        self._target_zone = value

    @builtins.property
    def target_pose(self):
        """Message field 'target_pose'."""
        return self._target_pose

    @target_pose.setter
    def target_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'target_pose' field must be a sub message of type 'Pose'"
        self._target_pose = value

    @builtins.property
    def named_pose(self):
        """Message field 'named_pose'."""
        return self._named_pose

    @named_pose.setter
    def named_pose(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'named_pose' field must be of type 'str'"
        self._named_pose = value

    @builtins.property
    def gripper_close(self):
        """Message field 'gripper_close'."""
        return self._gripper_close

    @gripper_close.setter
    def gripper_close(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'gripper_close' field must be of type 'bool'"
        self._gripper_close = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ArmControl_Response(type):
    """Metaclass of message 'ArmControl_Response'."""

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
            module = import_type_support('python_moveit_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'python_moveit_interface.srv.ArmControl_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__arm_control__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__arm_control__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__arm_control__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__arm_control__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__arm_control__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ArmControl_Response(metaclass=Metaclass_ArmControl_Response):
    """Message class 'ArmControl_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
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
        self.message = kwargs.get('message', str())

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
        if self.message != other.message:
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
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_ArmControl(type):
    """Metaclass of service 'ArmControl'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('python_moveit_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'python_moveit_interface.srv.ArmControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__arm_control

            from python_moveit_interface.srv import _arm_control
            if _arm_control.Metaclass_ArmControl_Request._TYPE_SUPPORT is None:
                _arm_control.Metaclass_ArmControl_Request.__import_type_support__()
            if _arm_control.Metaclass_ArmControl_Response._TYPE_SUPPORT is None:
                _arm_control.Metaclass_ArmControl_Response.__import_type_support__()


class ArmControl(metaclass=Metaclass_ArmControl):
    from python_moveit_interface.srv._arm_control import ArmControl_Request as Request
    from python_moveit_interface.srv._arm_control import ArmControl_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
