import numpy as np
import plus_general.utils.matrix_utils as matrix_utils
from builtins import list
import operator
from collections import OrderedDict
import pickle


class AnyObj(object):
    def __init__(self, value):
        self.value = value


class NoneObj(AnyObj):
    def __init__(self):
        super(NoneObj, self).__init__(None)
        

class SerializableClass(object):
    def __init__(self):
        self._recorded = OrderedDict()

    def __setattr__(self, key, value):
        _recorded = self.__dict__.get('_recorded')

        if _recorded is None and key == '_recorded':
            object.__setattr__(self, key, value)
            return

        if isinstance(value, SerializableClass):
            _recorded[key] = value
        elif isinstance(value, AnyObj):
            _recorded[key] = value.value
        elif key in self._recorded:
            _recorded[key] = value
        else:
            # super(SerializableClass, self).__setattr__(key, value)
            object.__setattr__(self, key, value)

    def __getattr__(self, item):
        _recorded = self.__dict__['_recorded']
        if item in _recorded:
            return _recorded[item]
        else:
            raise AttributeError("'{}' object has no attribute '{}'".format(
                type(self).__name__, item))

    def serialize(self):
        result = {}
        for key in self._recorded:
            value = self.__getattr__(key)
            if isinstance(value, SerializableClass):
                result[key] = value.serialize()
            else:
                result[key] = value
        return result

    def deserialize(self, data):
        for k in self._recorded:
            if k not in data:
                continue
            if isinstance(self.__getattr__(k), SerializableClass):
                self.__getattr__(k).deserialize(data[k])
            else:
                self.__setattr__(k, data[k])
        return self


class SerializableList(SerializableClass):
    def __init__(self, objects=None):
        super(SerializableList, self).__init__()
        if objects is not None:
            self.extend(objects)

    def __setattr__(self, key, value):
        # we record everything in list
        if not isinstance(value, (AnyObj, SerializableClass)):
            value = AnyObj(value)
        super(SerializableList, self).__setattr__(key, value)

    def __len__(self):
        return len(self._recorded)

    def _get_abs_string_index(self, idx):
        """Get the absolute index for the list of modules"""
        idx = operator.index(idx)
        if not (-len(self) <= idx < len(self)):
            raise IndexError('index {} is out of range'.format(idx))
        if idx < 0:
            idx += len(self)
        return str(idx)

    def append(self, item):
        self.__setattr__(str(len(self)), item)

    def extend(self, modules):
        offset = len(self)
        for i, module in enumerate(modules):
            self.__setattr__(str(offset + i), module)

    def __iter__(self):
        return iter(self._recorded.values())

    def __iadd__(self, modules):
        return self.extend(modules)

    def __getitem__(self, idx):
        if isinstance(idx, slice):
            return self.__class__(list(self._recorded.values())[idx])
        else:
            return self._modules[self._get_abs_string_index(idx)]

    def __setitem__(self, idx, module):
        idx = self._get_abs_string_index(idx)
        return self.__setattr__(str(idx), module)

    def insert(self, index: int, module) -> None:
        for i in range(len(self), index, -1):
            self._recorded[str(i)] = self._recorded[str(i - 1)]
        self._recorded[str(index)] = module


class SerializableDict(SerializableClass, dict):
    def __init__(self):
        super(SerializableDict, self).__init__()

    def serialize(self):
        result = {}
        for k, v in self.items():
            # if '__recorded' in k:
            #     continue

            if isinstance(v, SerializableClass):
                result[k] = v.serialize()
            else:
                result[k] = v
        return result

    def deserialize(self, data):
        for k, v in data.items():
            self[k] = v
        return self


# class KeyPoint(SerializableClass):
#     def __init__(self):
#         super(KeyPoint, self).__init__()
#         self.id = NoneObj()
#         self.frame_id = NoneObj()
#         self.pi = NoneObj() # image point 2d
#         self.pc = NoneObj() # camera point 3d
#         self.pg = NoneObj() # global point 3d
#         self.pr = NoneObj()
#         self.desc = NoneObj()
#         self.matched = AnyObj(False)
#
#
# class Frame(SerializableClass):
#     def __init__(self):
#         super(Frame, self).__init__()
#         self.id = NoneObj()
#         self.pose = NoneObj()
#         self.keypoints = SerializableDict()
#         self.odom_pose = NoneObj()
#         self.odom_cam_pose = NoneObj()
#         self.timestamp = NoneObj()
#
#     def deserialize(self, data):
#         super(Frame, self).deserialize(data)
#         for k, v in self.keypoints.items():
#             self.keypoints[k] = KeyPoint().deserialize(v)
#
#     def points_to_global(self):
#         local_to_world = np.linalg.inv(self.pose)
#         for kpt in self.keypoints.values():
#             kpt.pg = matrix_utils.homo_mul(kpt.pc.reshape((1, 3)), local_to_world, False)[0]


class KeyPoint(object):
    def __init__(self):
        self.id = None
        self.frame_id = None
        self.pi = None # image point 2d
        self.pc = None # camera point 3d
        self.pg = None # global point 3d
        self.pr = None
        self.desc = None
        self.matched = False

    def serialize(self):
        return self.__dict__

    def deserialize(self, data):
        for k, v in data.items():
            self.__setattr__(k, v)
        return self


class Frame(object):
    def __init__(self):
        self.id = None
        self.pose = None
        self.keypoints = {}
        self.odom_pose = None
        self.odom_cam_pose = None
        self.timestamp = None

    def serialize(self):
        data = self.__dict__
        result = {}
        for k, v in data.items():
            if k == 'keypoints':
                result[k] = {k: v.serialize() for k, v in data[k].items()}
            else:
                result[k] = v
        return result

    def deserialize(self, data):
        for k, v in data.items():
            if k == 'keypoints':
                self.keypoints = {k: KeyPoint().deserialize(v) for k, v in data[k].items()}
            else:
                self.__setattr__(k, v)
        return self

    def points_to_global(self):
        local_to_world = np.linalg.inv(self.pose)
        for kpt in self.keypoints.values():
            kpt.pg = matrix_utils.homo_mul(kpt.pc.reshape((1, 3)), local_to_world, False)[0]


# class KeyPoint(object):
#     def __init__(self):
#         self.id = None
#         self.frame_id = None
#         self.pi = None # image point 2d
#         self.pc = None # camera point 3d
#         self.pg = None # global point 3d
#         self.pr = None
#         self.desc = None
#         self.matched = False
#
#     def to_dict(self):
#         return {
#             "id": self.id,
#             "frame_id": self.frame_id,
#             "pi": self.pi,
#             "pc": self.pc,
#             "pr": self.pr,
#             "pg": self.pg,
#             "desc": self.desc
#         }
#
#     def from_dict(self, data):
#         for k, v in data.items():
#             self.__setattr__(k, v)
#         return self
#
#
# class Frame(object):
#     def __init__(self):
#         self.id = None
#         self.pose = None
#         self.keypoints = {}
#         self.odom_pose = None
#         self.odom_cam_pose = None
#         self.timestamp = None
#
#     def to_dict(self):
#         return {
#             "id": self.id,
#             "pose": self.pose,
#             "keypoints": {k: v.to_dict() for k, v in self.keypoints.items()},
#             "odom_pose": self.odom_pose,
#             "odom_cam_pose": self.odom_cam_pose,
#             "timestamp": self.timestamp
#         }
#
#     def from_dict(self, data):
#         for k, v in data.items():
#             if k == 'keypoints':
#                 self.keypoints = {k: KeyPoint().from_dict(v) for k, v in v.items()}
#             else:
#                 self.__setattr__(k, v)
#         return self
#
#     def points_to_global(self):
#         local_to_world = np.linalg.inv(self.pose)
#         for kpt in self.keypoints.values():
#             kpt.pg = matrix_utils.homo_mul(kpt.pc.reshape((1, 3)), local_to_world, False)[0]


def save_frame(frame, path):
    data = frame.serialize()
    with open(path, 'wb') as f:
        pickle.dump(data, f)


def load_frame(path):
    if path.endswith('.pth'):
        import torch
        data = torch.load(path, encoding='latin1')
    elif path.endswith('.pkl'):
        with open(path, 'rb') as f:
            data = pickle.load(f)
    else:
        raise NotImplementedError

    frame = Frame()
    frame.deserialize(data)
    return frame


class IDGenerator(object):
    def __init__(self, init=-1):
        self.id = init

    def generate(self):
        self.id += 1
        return self.id

    def clear(self):
        self.id = -1


if __name__ == '__main__':
    pass

