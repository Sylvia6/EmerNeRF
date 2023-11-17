import struct

_struct_I = struct.Struct('<I')
_struct_3I = struct.Struct("<3I")


class Header(object):
  __slots__ = ['seq','stamp','frame_id']
  _slot_types = ['uint32','time','string']

  def __init__(self, *args, **kwds):
      self.seq = 0
      self.stamp = None
      self.frame_id = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
      _x = self
      buff.write(_struct_3I.pack(_x.seq, _x.stamp.secs, _x.stamp.nsecs))
      _x = self.frame_id
      length = len(_x)
      # if python3 or type(_x) == unicode:
      #   _x = _x.encode('utf-8')
      #   length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))

  def deserialize(self, str):
      if self.stamp is None:
        self.stamp = Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.seq, _x.stamp.secs, _x.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      # if python3:
      #   self.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      # else:
      self.frame_id = str[start:end]
      self.stamp.canon()
      return self


def deserialize_string(str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """


    end = 0
    start = end
    end += 4
    (length,) = _struct_I.unpack(str[start:end])
    start = end
    end += length

    data = str[start:end]
    return data


def deserialize_compress_image(str):

    # if self.header is None:
    #     self.header = std_msgs.msg.Header()
    end = 0
    # _x = self
    start = end
    end += 12
    # (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I().unpack(str[start:end])
    start = end
    end += 4
    (length,) = _struct_I.unpack(str[start:end])
    start = end
    end += length
    # if python3:
    #     self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
    # else:
    #     self.header.frame_id = str[start:end]
    start = end
    end += 4
    (length,) = _struct_I.unpack(str[start:end])
    start = end
    end += length
    # if python3:
    #     self.format = str[start:end].decode('utf-8', 'rosmsg')
    # else:
    #     self.format = str[start:end]
    start = end
    end += 4
    (length,) = _struct_I.unpack(str[start:end])
    start = end
    end += length
    data = str[start:end]
    return data

