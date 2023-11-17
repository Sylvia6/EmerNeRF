import math

import cv2
import numpy as np
import os
from plus_general.utils.matrix_utils import homo_mul
from plus_general.utils.pcl_utils import write_color_pcd
import torch.nn.functional as F
import torch
from plus_general.utils.calibrator import Calibrator, SearchPolicy


class PathManager(object):
    class Dir(object):
        def __init__(self, name=None):
            self.name = name

    class File(object):
        def __init__(self, name=None):
            self.name = name

    def __init__(self, cache_pre, bag_name=None, post_fix=None):
        self.__dir_set = set()
        self.__file_set = set()
        self.cache_pre = cache_pre
        self.cache_root = PathManager.Dir()
        self.stereo_cache_root = PathManager.Dir()
        self.pose_path = PathManager.File()
        self.stereo_file_path = PathManager.File()
        self.lane_save_path = PathManager.File()
        self.fix_lane_save_path = PathManager.File()
        self.lane_cloud_root = PathManager.Dir()
        self.cloud_root = PathManager.Dir()
        self.curb_cloud_root = PathManager.Dir()
        self.frame_root = PathManager.Dir()
        if bag_name is not None:
            self.init(bag_name, post_fix)

    def __setattr__(self, key, value):
        if isinstance(value, PathManager.Dir):
            self.__dir_set.add(key)
            super(PathManager, self).__setattr__(key, value.name)
        elif isinstance(value, PathManager.File):
            self.__file_set.add(key)
            super(PathManager, self).__setattr__(key, value.name)
        else:
            super(PathManager, self).__setattr__(key, value)

    def join_root(self, *args):
        return os.path.join((self.cache_root), *args)

    def make_dir(self, path):
        if not os.path.exists(path):
            os.makedirs(path)

    def make_all_dirs(self):
        for attr in self.__dir_set:
            dir_name = self.__getattribute__(attr)
            if dir_name is not None:
                self.make_dir(dir_name)

    def init(self, bag_name, post_fix=None):
        self.bag_name = bag_name
        self.cache_root = os.path.join(self.cache_pre, os.path.basename(bag_name))
        if post_fix is not None:
            self.cache_root += post_fix
        self.cache_root = self.cache_root.replace('.db','')
        self.stereo_cache_root = self.join_root('stereo')
        self.pose_path = self.join_root('pose_results.pkl')
        self.stereo_file_path = self.join_root('stereo.yaml')
        self.lane_save_path = self.join_root("lanes.pth")
        self.fix_lane_save_path = self.join_root("fix_lanes.pth")
        self.cloud_root = self.join_root('cloud')
        self.curb_cloud_root = self.join_root('curb_cloud')
        self.frame_root = self.join_root('frame')
        self.make_all_dirs()


def max_filter(image, ks=3):
    kernel = np.ones((ks, ks), np.uint8)
    img = cv2.dilate(image, kernel, iterations=1)
    return img


def make_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)


def vis_disparity(disp):
    return disp / np.max(disp)


def disparity_warp(right, disp):
    # mask = disp < 0
    offset = np.zeros((right.shape[0], right.shape[1], 2))
    offset[..., 0] -= disp
    offset[:,:,0] += np.arange(right.shape[1])
    offset[:,:,1] += np.arange(right.shape[0])[:, np.newaxis]
    res = cv2.remap(right, offset.astype('float32'), None, cv2.INTER_LINEAR)
    return res


def render_keypoints(image, points):
    image = image.copy()
    for x, y in points:
        x = int(x)
        y = int(y)
        cv2.circle(image, (x, y), 3, (0, 255, 0), -1)
    return image


def generate_obstacle_mask(im_h, im_w, pb, scale=1.0):
    mask = np.zeros((im_h, im_w))
    for obs in pb.obstacle:
        if obs.source_camera_id == 1:
            x = obs.image_cx
            y = obs.image_cy
            w = obs.box_width * scale
            h = obs.box_height 

            valid = True
            for n in [x, y, h, w]:
                if math.isinf(n):
                    valid = False
                    break
            if not valid:
                continue
            x0 = max(0, int(x-w/2))
            y0 = max(0, int(y-h/2))
            x1 = min(im_w, int(x+w/2))
            y1 = min(im_h, int(y+h/2+h*(scale-1.0)))
            mask[y0:y1, x0:x1] = 1
    return mask


def extract_single_point_cloud(image, mat_3d, pose=None, dst_size=None, spatial_stride=1, mask=None, roi=None):
    if roi is None:
        roi = [-15, 15, -10, 10, 15, 60]

    if dst_size is not None:
        image = cv2.resize(image, dst_size)
        mat_3d = cv2.resize(mat_3d, dst_size)
    row_stride = spatial_stride
    col_stride = spatial_stride
    # disparity_mask = points_3d_mat_mask
    result_3d_mat = mat_3d[::row_stride, ::col_stride]
    # flow = flow[::row_stride, ::col_stride]
    stride_image = image[::row_stride, ::col_stride]
    if mask is not None:
        disparity_mask = mask[::row_stride, ::col_stride]
    else:
        disparity_mask = np.ones(stride_image.shape[:2])
    # disparity_mask = disparity_mask[::row_stride, ::col_stride]
    disparity_mask = disparity_mask * (result_3d_mat[..., 0] < roi[1])
    disparity_mask = disparity_mask * (result_3d_mat[..., 0] > roi[0])
    disparity_mask = disparity_mask * (result_3d_mat[..., 1] < roi[3])
    disparity_mask = disparity_mask * (result_3d_mat[..., 1] > roi[2])
    disparity_mask = disparity_mask * (result_3d_mat[..., 2] > roi[4])
    disparity_mask = disparity_mask * (result_3d_mat[..., 2] < roi[5])

    points_3d = result_3d_mat[disparity_mask > 0]
    if len(points_3d) == 0:
        return [], []
    points_color = stride_image[disparity_mask > 0]
    # matrix = np.linalg.inv(pose)
    if pose:
        points_3d = homo_mul(points_3d, pose, False)
    return points_3d, points_color


def extract_point_cloud(stereo_cache_root, results, save_root=None, file_stride=1, spatial_stride=2, dst_size=None):
    all_colors = []
    all_points = []
    file_idx = -1
    for id, t, pose in results:
        im_path = os.path.join(stereo_cache_root, "{}.jpg".format(id))
        mat_path = os.path.join(stereo_cache_root, "{}.npy".format(id))
        if not os.path.exists(mat_path):
            continue
        file_idx += 1
        if file_stride % file_stride != 0:
            continue
        image = cv2.imread(im_path)
        mat_3d = np.load(mat_path)

        if dst_size is not None:
            image = cv2.resize(image, dst_size)
            mat_3d = cv2.resize(mat_3d, dst_size)

        row_stride = spatial_stride
        col_stride = spatial_stride
        # disparity_mask = points_3d_mat_mask
        result_3d_mat = mat_3d[::row_stride, ::col_stride]
        # flow = flow[::row_stride, ::col_stride]
        stride_image = image[::row_stride, ::col_stride]
        disparity_mask = np.ones(stride_image.shape[:2])
        # disparity_mask = disparity_mask[::row_stride, ::col_stride]
        disparity_mask = disparity_mask * (result_3d_mat[..., 0] < 12)
        disparity_mask = disparity_mask * (result_3d_mat[..., 0] > -12)
        disparity_mask = disparity_mask * (result_3d_mat[..., 1] < 3)
        disparity_mask = disparity_mask * (result_3d_mat[..., 1] > -3)
        disparity_mask = disparity_mask * (result_3d_mat[..., 2] > 25)
        disparity_mask = disparity_mask * (result_3d_mat[..., 2] < 50)

        points_3d = result_3d_mat[disparity_mask > 0]
        points_color = stride_image[disparity_mask > 0]
        if save_root is not None:
            save_path = os.path.join(save_root, "{:05d}.pcd".format(id))
            write_color_pcd(points_3d, points_color, save_path)
        # matrix = np.linalg.inv(pose)
        matrix = pose
        points_3d = homo_mul(points_3d, matrix, False)
        all_colors.append(points_color)
        all_points.append(points_3d)

    all_points = np.concatenate(all_points, 0)
    all_colors = np.concatenate(all_colors, 0)
    return all_points, all_colors


def load_mat_3d(mat_path, image_shape=None):
    disp_data = np.load(mat_path)
    Q = disp_data['q']
    disp = disp_data['disp'].astype('float32')
    scale = 1
    if image_shape is not None:
        scale = image_shape[1] / float(disp.shape[1])
    disp = F.interpolate(torch.from_numpy(disp)[None].unsqueeze(1), size=image_shape,
                         mode='bilinear',
                         align_corners=True).squeeze(1)[0].numpy()
    disp *= scale
    mat_3d = cv2.reprojectImageTo3D(disp.astype('float32'), Q)
    return mat_3d


def get_stereo_calibrator(bag_path, drive_dir=None, calib_db_dir=None):
    calibrator = Calibrator.from_bag_message(bag_path)
    if 'stereo' not in calibrator.calib_params_dict:
        calibrator = Calibrator.from_bagname(bag_path, SearchPolicy.GUESS,
                                             drive_dir=drive_dir, calib_db_dir=calib_db_dir)

        if 'stereo' not in calibrator.calib_params_dict:
            raise ValueError('cannot find stereo calibration for bag {}'.format(bag_path))
    return calibrator


def get_calibrator(bag_path, drive_dir=None, calib_db_dir=None, rear=True, side=True):
    calibrator = Calibrator.from_bag_message(bag_path)
    if 'stereo' not in calibrator.calib_params_dict:
        calibrator = Calibrator.from_bagname(bag_path, SearchPolicy.GUESS,
                                             drive_dir=drive_dir, calib_db_dir=calib_db_dir,
                                             use_rear=rear, use_side=side)

        if 'stereo' not in calibrator.calib_params_dict:
            raise ValueError('cannot find stereo calibration for bag {}'.format(bag_path))
    return calibrator


def clip_line_max(points, max_x, dim=0):
    result = []
    for edge_idx in range(len(points)-1):
        pt0 = points[edge_idx]
        pt1 = points[edge_idx+1]

        if pt0[dim] < max_x and pt1[dim] < max_x:
            if len(result) == 0:
                result.append(pt0)
            result.append(pt1)
        elif pt0[dim] > max_x and pt1[dim] > max_x:
            continue
        else:
            vec = pt1 - pt0
            norm_vec = vec / (pt1[dim]-pt0[dim])
            dst_pt = pt0 + norm_vec * (max_x-pt0[dim])
            if len(result) == 0:
                result.append(pt0)
            result.append(dst_pt)
    return result


def clip_line_min(points, min_x, dim=0):
    result = []
    for edge_idx in range(len(points)-1):
        pt0 = points[edge_idx]
        pt1 = points[edge_idx+1]

        if pt0[dim] > min_x and pt1[dim] > min_x:
            if len(result) == 0:
                result.append(pt0)
            result.append(pt1)
        elif pt0[dim] < min_x and pt1[dim] < min_x:
            continue
        else:
            vec = pt1 - pt0
            norm_vec = vec / (pt1[dim]-pt0[dim])
            dst_pt = pt0 + norm_vec * (min_x-pt0[dim])
            result.append(dst_pt)
            if len(result) == 1:
                result.append(pt1)
    return result


def clip_line(points, min_v, max_v, dim=0):
    if min_v is not None:
        points = clip_line_min(points, min_v, dim)
    if max_v is not None:
        points = clip_line_max(points, max_v, dim)
    return points
