import struct
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
from .genpy import struct_I, Message
from . import genpy as genpy

_struct_I = struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I

_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I

_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d

_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d

_struct_6d = None
def _get_struct_6d():
    global _struct_6d
    if _struct_6d is None:
        _struct_6d = struct.Struct("<6d")
    return _struct_6d

_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d

_struct_36d = None
def _get_struct_36d():
    global _struct_36d
    if _struct_36d is None:
        _struct_36d = struct.Struct("<36d")
    return _struct_36d

_struct_IBI = None
def _get_struct_IBI():
    global _struct_IBI
    if _struct_IBI is None:
        _struct_IBI = struct.Struct("<IBI")
    return _struct_IBI

_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I

_struct_B2I = None
def _get_struct_B2I():
    global _struct_B2I
    if _struct_B2I is None:
        _struct_B2I = struct.Struct("<B2I")
    return _struct_B2I

_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B

class Point(Message):
  _type = "geometry_msgs/Point"
  _has_header = False  # flag to mark the presence of a Header object

  __slots__ = ['x','y','z']
  _slot_types = ['float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x,y,z

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Point, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
    else:
      self.x = 0.
      self.y = 0.
      self.z = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 24
      (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class Vector3(Message):
  _type = "geometry_msgs/Vector3"
  _has_header = False  # flag to mark the presence of a Header object
  __slots__ = ['x','y','z']
  _slot_types = ['float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x,y,z

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Vector3, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
    else:
      self.x = 0.
      self.y = 0.
      self.z = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 24
      (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class Quaternion(Message):
  _md5sum = "a779879fadf0160734f906b8c19c7004"
  _type = "geometry_msgs/Quaternion"
  _has_header = False  # flag to mark the presence of a Header object

  __slots__ = ['x','y','z','w']
  _slot_types = ['float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       x,y,z,w

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Quaternion, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.w is None:
        self.w = 0.
    else:
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.w = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 32
      (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class Pose(Message):
  _type = "geometry_msgs/Pose"
  _has_header = False  # flag to mark the presence of a Header object
  __slots__ = ['position','orientation']
  _slot_types = ['geometry_msgs/Point','geometry_msgs/Quaternion']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       position,orientation

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Pose, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.position is None:
        self.position = Point()
      if self.orientation is None:
        self.orientation = Quaternion()
    else:
      self.position = Point()
      self.orientation = Quaternion()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.position is None:
        self.position = Point()
      if self.orientation is None:
        self.orientation = Quaternion()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.position.x, _x.position.y, _x.position.z, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class Twist(Message):
  _type = "geometry_msgs/Twist"
  _has_header = False  # flag to mark the presence of a Header object

  __slots__ = ['linear','angular']
  _slot_types = ['geometry_msgs/Vector3','geometry_msgs/Vector3']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       linear,angular

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Twist, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.linear is None:
        self.linear = Vector3()
      if self.angular is None:
        self.angular = Vector3()
    else:
      self.linear = Vector3()
      self.angular = Vector3()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.linear is None:
        self.linear = Vector3()
      if self.angular is None:
        self.angular = Vector3()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.linear.x, _x.linear.y, _x.linear.z, _x.angular.x, _x.angular.y, _x.angular.z,) = _get_struct_6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class PoseWithCovariance(Message):
  _type = "geometry_msgs/PoseWithCovariance"
  _has_header = False  # flag to mark the presence of a Header object

  __slots__ = ['pose','covariance']
  _slot_types = ['geometry_msgs/Pose','float64[36]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pose,covariance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PoseWithCovariance, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.pose is None:
        self.pose = Pose()
      if self.covariance is None:
        self.covariance = [0.] * 36
    else:
      self.pose = Pose()
      self.covariance = [0.] * 36

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.pose is None:
        self.pose = Pose()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 288
      self.covariance = _get_struct_36d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class TwistWithCovariance(Message):
  _type = "geometry_msgs/TwistWithCovariance"
  _has_header = False  # flag to mark the presence of a Header object
  __slots__ = ['twist','covariance']
  _slot_types = ['geometry_msgs/Twist','float64[36]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       twist,covariance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TwistWithCovariance, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.twist is None:
        self.twist = Twist()
      if self.covariance is None:
        self.covariance = [0.] * 36
    else:
      self.twist = Twist()
      self.covariance = [0.] * 36

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.twist is None:
        self.twist = Twist()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.twist.linear.x, _x.twist.linear.y, _x.twist.linear.z, _x.twist.angular.x, _x.twist.angular.y, _x.twist.angular.z,) = _get_struct_6d().unpack(str[start:end])
      start = end
      end += 288
      self.covariance = _get_struct_36d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class Header(Message):
  _type = "std_msgs/Header"
  _has_header = False  # flag to mark the presence of a Header object

  __slots__ = ['seq','stamp','frame_id']
  _slot_types = ['uint32','time','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       seq,stamp,frame_id

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Header, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.seq is None:
        self.seq = 0
      if self.stamp is None:
        self.stamp = genpy.Time()
      if self.frame_id is None:
        self.frame_id = ''
    else:
      self.seq = 0
      self.stamp = genpy.Time()
      self.frame_id = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.seq, _x.stamp.secs, _x.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.frame_id = str[start:end]
      self.stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class Odometry(Message):
  _type = "nav_msgs/Odometry"
  _has_header = True  # flag to mark the presence of a Header object

  __slots__ = ['header','child_frame_id','pose','twist']
  _slot_types = ['std_msgs/Header','string','geometry_msgs/PoseWithCovariance','geometry_msgs/TwistWithCovariance']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,child_frame_id,pose,twist

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Odometry, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = Header()
      if self.child_frame_id is None:
        self.child_frame_id = ''
      if self.pose is None:
        self.pose = PoseWithCovariance()
      if self.twist is None:
        self.twist = TwistWithCovariance()
    else:
      self.header = Header()
      self.child_frame_id = ''
      self.pose = PoseWithCovariance()
      self.twist = TwistWithCovariance()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = Header()
      if self.pose is None:
        self.pose = PoseWithCovariance()
      if self.twist is None:
        self.twist = TwistWithCovariance()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.child_frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.child_frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.pose.pose.position.x, _x.pose.pose.position.y, _x.pose.pose.position.z, _x.pose.pose.orientation.x, _x.pose.pose.orientation.y, _x.pose.pose.orientation.z, _x.pose.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 288
      self.pose.covariance = _get_struct_36d().unpack(str[start:end])
      _x = self
      start = end
      end += 48
      (_x.twist.twist.linear.x, _x.twist.twist.linear.y, _x.twist.twist.linear.z, _x.twist.twist.angular.x, _x.twist.twist.angular.y, _x.twist.twist.angular.z,) = _get_struct_6d().unpack(str[start:end])
      start = end
      end += 288
      self.twist.covariance = _get_struct_36d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class CompressedImage(Message):
  _type = "sensor_msgs/CompressedImage"
  _has_header = True  # flag to mark the presence of a Header object

  __slots__ = ['header','format','data']
  _slot_types = ['std_msgs/Header','string','uint8[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,format,data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CompressedImage, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = Header()
      if self.format is None:
        self.format = ''
      if self.data is None:
        self.data = b''
    else:
      self.header = Header()
      self.format = ''
      self.data = b''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.format = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.format = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.data = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class String(Message):
  _type = "std_msgs/String"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """string data
"""
  __slots__ = ['data']
  _slot_types = ['string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(String, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.data is None:
        self.data = ''
    else:
      self.data = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    _x = self.data
    length = len(_x)
    # if python3 or type(_x) == unicode:
    #   _x = _x.encode('utf-8')
    #   length = len(_x)
    buff.write(struct.Struct('<I%ss'%length).pack(length, _x))


  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        # self.data = str[start:end].decode('utf-8', 'rosmsg')
        self.data = str[start:end]
      else:
        self.data = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


class PointField(genpy.Message):
  _md5sum = "268eacb2962780ceac86cbd17e328150"
  _type = "sensor_msgs/PointField"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
"""
  # Pseudo-constants
  INT8 = 1
  UINT8 = 2
  INT16 = 3
  UINT16 = 4
  INT32 = 5
  UINT32 = 6
  FLOAT32 = 7
  FLOAT64 = 8

  __slots__ = ['name','offset','datatype','count']
  _slot_types = ['string','uint32','uint8','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       name,offset,datatype,count

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PointField, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.name is None:
        self.name = ''
      if self.offset is None:
        self.offset = 0
      if self.datatype is None:
        self.datatype = 0
      if self.count is None:
        self.count = 0
    else:
      self.name = ''
      self.offset = 0
      self.datatype = 0
      self.count = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_IBI().pack(_x.offset, _x.datatype, _x.count))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      _x = self
      start = end
      end += 9
      (_x.offset, _x.datatype, _x.count,) = _get_struct_IBI().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


class PointCloud2(genpy.Message):
  _md5sum = "1158d486dd51d683ce2f1be655c3c181"
  _type = "sensor_msgs/PointCloud2"
  _has_header = True #flag to mark the presence of a Header object
  __slots__ = ['header','height','width','fields','is_bigendian','point_step','row_step','data','is_dense']
  _slot_types = ['std_msgs/Header','uint32','uint32','sensor_msgs/PointField[]','bool','uint32','uint32','uint8[]','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,height,width,fields,is_bigendian,point_step,row_step,data,is_dense

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PointCloud2, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = Header()
      if self.height is None:
        self.height = 0
      if self.width is None:
        self.width = 0
      if self.fields is None:
        self.fields = []
      if self.is_bigendian is None:
        self.is_bigendian = False
      if self.point_step is None:
        self.point_step = 0
      if self.row_step is None:
        self.row_step = 0
      if self.data is None:
        self.data = b''
      if self.is_dense is None:
        self.is_dense = False
    else:
      self.header = Header()
      self.height = 0
      self.width = 0
      self.fields = []
      self.is_bigendian = False
      self.point_step = 0
      self.row_step = 0
      self.data = b''
      self.is_dense = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = Header()
      if self.fields is None:
        self.fields = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.height, _x.width,) = _get_struct_2I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.fields = []
      for i in range(0, length):
        val1 = PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _get_struct_IBI().unpack(str[start:end])
        self.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.is_bigendian, _x.point_step, _x.row_step,) = _get_struct_B2I().unpack(str[start:end])
      self.is_bigendian = bool(self.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.data = str[start:end]
      start = end
      end += 1
      (self.is_dense,) = _get_struct_B().unpack(str[start:end])
      self.is_dense = bool(self.is_dense)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill