import cv2
import os

import numpy as np
import yaml
import glob
import plus_general.utils.calibration_utils as conf_utils
from plus_general.utils.calibration_utils import default_calib_db_dir, default_drive_dir
from plus_general.utils.ros_utils import deserialize_string

# cur_path = os.path.dirname(os.path.realpath(__file__))
# cfg_path = os.path.join(cur_path, '..', 'project_config.yaml')
# drive_dir = ""
# common_py_dir = ""
# with open(cfg_path) as f:
#     project_cfg_data = yaml.safe_load(f)
#     drive_dir = project_cfg_data['drive_dir']
#     common_py_dir = project_cfg_data['common_py']
# drive_dir = ""
# common_py_dir = "/home/plusai/ysy/work/common_py/python"


default_calib_key = ['lane_camera', 'stereo', 'camera_rear_left', 'camera_rear_right', 'side_left', 'side_right']
# calib_config_search_keys = {
#     'lane_camera': ['sensor_calib', 'lane_camera_calib_name'],
#     'stereo': ['sensor_calib', 'stereo_calib_name'],
#     'camera_rear_left': ['sensor_calib', 'camera_rear_left_calib_name'],
#     'camera_rear_right': ['sensor_calib', 'camera_rear_right_calib_name'],
# }


def unwarp(image, size, M, D, R, P):
    h, w, c = image.shape
    origin_size = (w, h)
    if size != origin_size:
        image = cv2.resize(image, size, interpolation=cv2.INTER_LINEAR)
    map1, map2 = cv2.initUndistortRectifyMap(M, D, R, P, size, cv2.CV_16SC2)
    image_rectified = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
    return image_rectified


def unwarp_map(image, size, map1, map2, rotate=-1):
    h, w, c = image.shape
    origin_size = (w, h)


    if size != origin_size:
        image = cv2.resize(image, size, interpolation=cv2.INTER_LINEAR)

    image_rectified = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
    if rotate != -1:
        image_rectified = cv2.rotate(image_rectified, rotate)
    return image_rectified


def generate_map(size, M, D, R, P):
    map1, map2 = cv2.initUndistortRectifyMap(M, D, R, P, size, cv2.CV_16SC2)
    return map1, map2


def generate_fisheye_map(size, M, D, R, P):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(M, D, R, P, size, cv2.CV_16SC2)
    return map1, map2

class SearchPolicy:
    DEFAULT = 0 # use the default config
    SPECIFY = 1 # by user
    GUESS = 2


class Calibrator:
    def __init__(self, calib_params, calib_key=None):
        self.calib_params_dict = calib_params
        self.map_cache = {}
        # if calib_key is None:
        #     self.calib_key = default_calib_key
        # else:
        #     self.calib_key = calib_key
        self.calib_key = list(self.calib_params_dict.keys())
        self.generate_map_cache()

    def __getitem__(self, item):
        return self.calib_params_dict[item]

    def generate_map_cache(self):
        for key in self.calib_params_dict.keys():
            if key == 'stereo':
                self.map_cache['stereo'] = []
                size = (self.calib_params_dict[key]['width'], self.calib_params_dict[key]['height'])
                M, D, R, P = [self.calib_params_dict[key][k] for k in ['M1', 'D1', 'R1', 'P1']]
                self.map_cache['stereo'].append(generate_map(size, M, D, R, P))
                M, D, R, P = [self.calib_params_dict[key][k] for k in ['M2', 'D2', 'R2', 'P2']]
                self.map_cache['stereo'].append(generate_map(size, M, D, R, P))
            else:
                try:
                    size = (self.calib_params_dict[key]['width'], self.calib_params_dict[key]['height'])
                    M, D, R, P = [self.calib_params_dict[key][k] for k in ['M', 'D', 'R', 'P']]
                    if key in ['side_left', 'side_right']:
                        self.map_cache[key] = generate_fisheye_map(size, M, D, R, P)
                    else:
                        self.map_cache[key] = generate_map(size, M, D, R, P)
                except Exception as e:
                    print("warning: cannot generate map cache for key {}".format(key))

    @classmethod
    def from_bagname(cls, bagname, search_policy, calib_key=None, drive_dir=None, calib_db_dir=None,
                     use_rear=True, use_side=True):
        if calib_key is None:
            calib_key = ['lane_camera', 'stereo']
            if use_rear:
                calib_key += ['camera_rear_left', 'camera_rear_right']
            if use_side:
                calib_key += ['side_left', 'side_right']
            # calib_key = default_calib_key
        name_infos = os.path.basename(bagname).split('_')
        timestamp = name_infos[0]
        car_name = name_infos[1]
        if name_infos[2].startswith("LFWSR"):
            car_name += "_"+name_infos[2]
        bag_date = None
        if len(timestamp) == 8:
            bag_date = timestamp
        elif 'T' in timestamp:
            bag_date = timestamp.split('T')[0]
        else:
            raise IOError('Unknown bagname: {}'.format(bagname))

        return cls.from_car_date2(car_name, bag_date, search_policy, calib_key,
                                drive_dir=drive_dir, calib_db_dir=calib_db_dir)

    @staticmethod
    def search_config(config_dir, car, reference_date):
        possibles = glob.glob(os.path.join(config_dir, 'common_config.prototxt.{}-*'.format(car)))
        max_date = None
        max_idx = None
        for idx, p in enumerate(possibles):
            date = p[-8:]
            if date < reference_date:
                if max_date is None or max_date < date:
                    max_date = date
                    max_idx = idx

        if max_date is None:
            return None
        else:
            # return os.path.join(config_dir, 'common_config.prototxt.{}-{}'.format(car, max_date))
            return possibles[max_idx]

    @classmethod
    def from_car_date(cls, car_name,
                      calib_date,
                      reference_date,
                      search_latest_before_data=False,
                      search_latest=False,
                      config_data=None,
                      config_before_reference=False,
                      calib_key=None,
                      drive_dir=None,
                      calib_db_dir=None):
        if drive_dir is None:
            drive_dir = default_drive_dir
        if calib_db_dir is None:
            calib_db_dir = default_calib_db_dir

        if calib_key is None:
            calib_key = default_calib_key

        if config_data:
            common_cfg_name = os.path.join(drive_dir, 'common', 'config', 'common_config.prototxt.{}-{}'.format(car_name, config_data))
        elif config_before_reference:
            common_cfg_name = Calibrator.search_config(os.path.join(drive_dir, 'common', 'config'), car_name, reference_date)
            if common_cfg_name is None:
                common_cfg_name = os.path.join(drive_dir, 'common', 'config', 'common_config.prototxt.{}'.format(car_name))
        else:
            common_cfg_name = os.path.join(drive_dir, 'common', 'config', 'common_config.prototxt.{}'.format(car_name))
        if not os.path.exists(common_cfg_name):
            raise IOError("common cfg file {} not found!".format(common_cfg_name))

        calib_file_dict = {k: {'name': None, 'date': None} for k in calib_key}
        with open(common_cfg_name) as f:
            lines = f.readlines()
            for line in lines:
                for k in calib_key:
                    name_key = k+'_calib_name'
                    date_key = k+'_calib_date'
                    if name_key in line:
                        calib_file_dict[k]['name'] = line.split('"')[1]
                    if date_key in line:
                        calib_file_dict[k]['date'] = line.split('"')[1]

        calib_params_dict = {}
        for k in calib_key:
            if calib_date is not None:
                dst_calib_date = calib_date
            else:
                if not search_latest and not search_latest_before_data:
                    dst_calib_date = calib_file_dict[k]['date']
                else:
                    dst_calib_date = reference_date
            calib_name = calib_file_dict[k]['name']
            calib_file = conf_utils.search_calib_file(car_name, calib_name, dst_calib_date,
                                                      search_latest_before_data, search_latest,
                                                      calib_db_dir=calib_db_dir)
            print("calib file for {} is {}".format(k, calib_file))
            if calib_file is None:
                raise IOError("invalid calib file!")
            if not os.path.exists(calib_file):
                raise IOError("calibration file {} not found!".format(calib_file))
            calib_params_dict[k] = conf_utils.load_opencv_yaml(calib_file)
        return cls(calib_params_dict, calib_key)

    def unwarp(self, image, key, idx=0, enable_rotate=False):
        assert key in self.calib_key, 'Unknown image key: {}!'.format(key)
        # if key == 'lane_camera':
        #     size = (self.calib_params_dict[key]['width'], self.calib_params_dict[key]['height'])
        #     M, D, R, P = [self.calib_params_dict[key][k] for k in ['M', 'D', 'R', 'P']]
        #     return unwarp(image, size, M, D, R, P)
        # if key == 'stereo':
        #     if idx not in [0, 1]:
        #         raise ValueError('stereo idx should be 0 (left) or 1 (right)')
        #     size = (self.calib_params_dict[key]['width'], self.calib_params_dict[key]['height'])
        #     if idx == 0:
        #         M, D, R, P = [self.calib_params_dict[key][k] for k in ['M1', 'D1', 'R1', 'P1']]
        #     else:
        #         M, D, R, P = [self.calib_params_dict[key][k] for k in ['M2', 'D2', 'R2', 'P2']]
        #     return unwarp(image, size, M, D, R, P)
        # else:
        #     size = (self.calib_params_dict[key]['width'], self.calib_params_dict[key]['height'])
        #     M, D, R, P = [self.calib_params_dict[key][k] for k in ['M', 'D', 'R', 'P']]
        #     return unwarp(image, size, M, D, R, P)
        if key == 'stereo':
            if idx not in [0, 1]:
                raise ValueError('stereo idx should be 0 (left) or 1 (right)')
            size = (self.calib_params_dict[key]['width'], self.calib_params_dict[key]['height'])
            map1 = self.map_cache[key][idx][0]
            map2 = self.map_cache[key][idx][1]
            rotate = self.calib_params_dict[key].get('rotate', -1)
            if not enable_rotate:
                rotate = -1
            return unwarp_map(image, size, map1, map2, rotate)
        else:
            size = (self.calib_params_dict[key]['width'], self.calib_params_dict[key]['height'])
            map1 = self.map_cache[key][0]
            map2 = self.map_cache[key][1]
            rotate = self.calib_params_dict[key].get('rotate', -1)
            if not enable_rotate:
                rotate = -1
            return unwarp_map(image, size, map1, map2, rotate)

    def unwarp_mono_by_cam_name(self, image, key, enable_rotate=False):
        dst_key, idx = None, None
        if key == 'front_left':
            # if 'lane_camera' in self.calib_params_dict:
            #     dst_key = 'lane_camera'
            #     idx = 0
            # else:
            dst_key = 'stereo'
            idx = 0
        elif key == 'front_right':
            dst_key = 'stereo'
            idx = 1
        elif key in ['rear_left', 'rear_right']:
            idx = 0
            if 'camera_' + key in self.calib_params_dict:
                dst_key = 'camera_' + key
            else:
                dst_key = key
        else:
            dst_key = key
            idx = 0
        return self.unwarp(image, dst_key, idx, enable_rotate)

    def undistort_points(self, points, key, idx=0, src_size=None, reverse=False):
        if key == 'stereo':
            if idx not in [0, 1]:
                raise ValueError('stereo idx should be 0 (left) or 1 (right)')
            size = (self.calib_params_dict[key]['width'], self.calib_params_dict[key]['height'])
            if idx == 0:
                M, D, R, P = [self.calib_params_dict[key][k] for k in ['M1', 'D1', 'R1', 'P1']]
            else:
                M, D, R, P = [self.calib_params_dict[key][k] for k in ['M2', 'D2', 'R2', 'P2']]
        else:
            size = (self.calib_params_dict[key]['width'], self.calib_params_dict[key]['height'])
            M, D, R, P = [self.calib_params_dict[key][k] for k in ['M', 'D', 'R', 'P']]

        if not reverse:
            if src_size is not None:
                points = points.copy()
                points[:, 0] *= size[0] / float(src_size[0])
                points[:, 1] *= size[1] / float(src_size[1])
            return cv2.undistortPoints(points, M, D, R=R, P=P)
        else:
            # http://docs.opencv.org/modules/imgproc/doc/geometric_transformations.html#cv.InitUndistortRectifyMap
            x = (points[:, 0] - P[0, 2]) / P[0, 0]
            y = (points[:, 1] - P[1, 2]) / P[1, 1]

            # rotate
            new_points = np.concatenate([x[..., None], y[..., None], np.ones((x.shape[0], 1))], 1)
            new_points = np.matmul(np.linalg.inv(np.array(R)), new_points.transpose((1, 0)))
            new_points = new_points / new_points[2]
            x = new_points[0]
            y = new_points[1]
            D = np.array(D).flatten()
            if D.shape[0] == 5:
                k1, k2, p1, p2, k3 = D[0], D[1], D[2], D[3], D[4]
            elif D.shape[0] == 4:
                k1, k2, p1, p2 = D[0], D[1], D[2], D[3]
                k3 = 0
            else:
                raise NotImplementedError("D={}".format(D))
            # radial distortion
            r2 = x * x + y * y
            x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
            y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)

            # tangential distortion
            x_distort = x_distort + (2. * p1 * x * y + p2 * (r2 + 2. * x * x))
            y_distort = y_distort + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y)

            x = x_distort * M[0, 0] + M[0, 2]
            y = y_distort * M[1, 1] + M[1, 2]
            points = np.concatenate((x[..., None], y[..., None]), 1)
            if src_size is not None:
                points[:, 0] *= float(src_size[0]) / size[0]
                points[:, 1] *= float(src_size[1]) / size[1]
            return points

    @classmethod
    def from_car_date2(cls, car_name, reference_date, search_policy, calib_key=None,
                       specify_cfg_name=None,drive_dir=None, calib_db_dir=None):
        if drive_dir is None:
            drive_dir = default_drive_dir
        if calib_db_dir is None:
            calib_db_dir = default_calib_db_dir

        if calib_key is None:
            calib_key = default_calib_key

        common_cfg_name = None
        if search_policy == SearchPolicy.DEFAULT:
            common_cfg_name = os.path.join(drive_dir, 'common', 'config', 'common_config.prototxt.{}'.format(car_name))

        elif search_policy == SearchPolicy.SPECIFY:
            assert specify_cfg_name is not None
            common_cfg_name = os.path.join(drive_dir,
                                           'common', 'config',
                                           'common_config.prototxt.{}-{}'.format(car_name, specify_cfg_name))

        elif search_policy == SearchPolicy.GUESS:
            config_dir = os.path.join(drive_dir, 'common', 'config')
            possible_config_files = glob.glob(os.path.join(config_dir, 'common_config.prototxt.{}*'.format(car_name)))
            possible_config_files.sort()
            dst_idx = None
            dst_max_cfg_date = None
            for idx, possible_cfg in enumerate(possible_config_files):
                cfg_max_cfg_date = None
                for key in calib_key:
                    # if key in calib_config_search_keys:
                    #     this_calib_date = conf_utils.grep_info_from_cfg(possible_cfg, calib_config_search_keys[key])
                    # else:
                    this_calib_date = conf_utils.grep_info_from_cfg(possible_cfg, ['sensor_calib', key + '_calib_date'])
                    if this_calib_date is None:
                        continue
                    cfg_max_cfg_date = this_calib_date if not cfg_max_cfg_date else max(cfg_max_cfg_date, this_calib_date)
                if cfg_max_cfg_date is None:
                    continue
                if cfg_max_cfg_date > reference_date:
                    continue
                if dst_idx is None or dst_max_cfg_date < cfg_max_cfg_date:
                    dst_idx = idx
                    dst_max_cfg_date = cfg_max_cfg_date
            if dst_idx is None:
                raise Exception("Cannot guess config file: carname={} reference_date={}".format(car_name, reference_date))
            common_cfg_name = possible_config_files[dst_idx]

        if common_cfg_name is None:
            raise Exception("common config is none!")

        if not os.path.exists(common_cfg_name):
            raise IOError("common config file {} not found!".format(common_cfg_name))

        calib_file_dict = {k: {'name': None, 'date': None} for k in calib_key}
        for k in calib_key:
            if 'side' in k:
                if k == 'side_left':
                    calib_file_dict[k]['name'] = 'side_left_camera'
                    calib_file_dict[k]['date'] = conf_utils.get_side_cam_info_from_cfg(common_cfg_name,
                                                                                                'side_left_camera')
                elif k == 'side_right':
                    calib_file_dict[k]['name'] = 'side_right_camera'
                    calib_file_dict[k]['date'] = conf_utils.get_side_cam_info_from_cfg(common_cfg_name,
                                                                                                'side_right_camera')
            else:
                calib_file_dict[k]['name'] = conf_utils.grep_info_from_cfg(common_cfg_name, ['sensor_calib', k+'_calib_name'])
                calib_file_dict[k]['date'] = conf_utils.grep_info_from_cfg(common_cfg_name, ['sensor_calib', k+'_calib_date'])

        calib_params_dict = {}
        key_mapping = {
            'camera_rear_left': 'rear_left',
            'camera_rear_right': 'rear_right',
        }
        for k in calib_key:
            calib_name = calib_file_dict[k]['name']
            calib_date = calib_file_dict[k]['date']
            calib_file = conf_utils.search_calib_file(car_name, calib_name, calib_date, False, False,
                                                      calib_db_dir=calib_db_dir)
            print("calib file for {} is {}".format(k, calib_file))
            if calib_file is None:
                raise IOError("invalid calib file")
            if not os.path.exists(calib_file):
                raise IOError("calibration file {} not found!".format(calib_file))
            if k in key_mapping:
                k = key_mapping[k]
            calib_params_dict[k] = conf_utils.load_opencv_yaml(calib_file)
        return cls(calib_params_dict, calib_key)

    @classmethod
    def from_cfg_dict(cls, cfg_dict):
        calib_key = []
        calib_params_dict = {}
        for k, calib_file in cfg_dict.items():
            if not os.path.exists(calib_file):
                raise IOError("calibration file {} not found!".format(calib_file))
            calib_params_dict[k] = conf_utils.load_opencv_yaml(calib_file)
            calib_key.append(k)
        return cls(calib_params_dict, calib_key)

    @classmethod
    def from_bag_message(cls, bag_path, mapping=None):
        import fastbag
        # from sensor_calibration.sensor_calibration_pb2 import CalibrationMessage
        # from csrc.common_proto_py import init_sys
        # init_sys()
        from plus_general.csrc.common_proto_py.sensor_calibration.sensor_calibration_pb2 import CalibrationMessage
        topics = ["/perception/calibrations", "/perception/ot_calibrations"]
        from plus_general.utils.bag_handler import get_bag
        src_bag = get_bag(bag_path)
        # src_bag.open()
        # bag_iter = src_bag.iter_messages(topics)


        calib_dict = {}
        for msg_topic, msg, ts in src_bag.msg_generator(topics, raw=True):
            if msg_topic in topics:
                calib_msg = CalibrationMessage()
                # bytes = msg.data.decode("utf-8")
                bytes = msg[1]
                bytes = deserialize_string(bytes)
                # bytes = str(msg.data, 'ascii')
                # a = bytes.decode('utf-8', 'rosmsg')
                # b = a.encode('utf-8', 'rosmsg')
                calib_msg.ParseFromString(bytes)
                # calib_msg.ParseFromString(msg[1])
                for calib_data in calib_msg.data:
                    calib_type = calib_data.calib_type
                    calib_data_str = calib_data.calib_data.encode("utf-8")
                    if not isinstance(calib_data_str, str):
                        calib_data_str = calib_data_str.decode("utf-8")
                    calib_data_dict = conf_utils.load_opencv_yaml_str(calib_data_str)
                    calib_dict[calib_data_dict['sensor_name']] = calib_data_dict
                topics.remove(msg_topic)
        # src_bag.close()

        if mapping is None:
            mapping = {
                "lane_camera": "front_left_camera",
                "stereo": "front_left_right_camera",
                "rear_left": "rear_left_camera",
                "rear_right": "rear_right_camera",
                "side_left": "side_left_camera",
                "side_right": "side_right_camera",
                "lidar": "lidar"
            }
        result_dict = {}
        calib_key = []
        for k, v in mapping.items():
            if v in calib_dict:
                result_dict[k] = calib_dict[v]
                calib_key.append(k)

        return cls(result_dict, calib_key)


if __name__ == '__main__':
    bag_name = "/home/plusai/Downloads/20201019T091805_j7-00007_0_710to770.bag"
    calibrator = Calibrator.from_bagname(bag_name, SearchPolicy.GUESS)
