import os
import torch
import torch.nn as nn
import cv2
from visual_odom.super_point import SuperPoint, sample_descriptors
from visual_odom.unimatch.inference import Inferencer
import numpy as np
from visual_odom.core import KeyPoint, Frame, IDGenerator
from visual_odom.utils import max_filter
from visual_odom.disflow import DisFlow
import plus_general.utils.matrix_utils as matrix_utils
from plus_general.utils.image_utils import ImageConcater
from plus_general.utils.flow_utils import draw_flow


def search_kpt_idx(kpt, kpt_idx_mat, search_rad):
    x, y = int(kpt.pi[0]), int(kpt.pi[[1]])
    min_x = max(x - search_rad, 0)
    max_x = min(x + search_rad, kpt_idx_mat.shape[1] - 1)
    min_y = max(y - search_rad, 0)
    max_y = min(y + search_rad, kpt_idx_mat.shape[0] - 1)
    dst_idx = -1
    best_distance = 10000000
    for r in range(min_y, max_y):
        for c in range(min_x, max_x):
            kpt_idx = kpt_idx_mat[r, c]
            if kpt_idx >= 0:
                distance = (r - y) ** 2 + (c - x) ** 2
                if distance < best_distance:
                    dst_idx = int(kpt_idx)
                    best_distance = distance
    return dst_idx


class SuperMatch(object):
    def __init__(self, nms_radius=2, max_keypoints=4096, dst_size=(960, 480), keypoint_threshold=0.01):
        self.super_point = SuperPoint({
            'nms_radius': nms_radius,
            'max_keypoints': max_keypoints,
            'keypoint_threshold': keypoint_threshold,
        })
        self.super_point.eval()
        self.super_point.cuda()
        self.dst_size = dst_size

    @torch.no_grad()
    def extract(self, image):
        if len(image.shape) == 3 and image.shape[2] == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("image", image)
        # cv2.waitKey()
        src_size = (image.shape[1], image.shape[0])
        image = cv2.resize(image, self.dst_size)
        image = image.astype('float32')
        # image = image * 2
        image = image/255
        image = image[None, None, ...]
        image = torch.from_numpy(image).cuda()
        result = self.super_point.forward({'image': image})
        scale_x, scale_y = src_size[0] / float(self.dst_size[0]), src_size[1] / float(self.dst_size[1])
        result['keypoints'][0][:, 0] *= scale_x
        result['keypoints'][0][:, 1] *= scale_y
        result['scale_x'] = scale_x
        result['scale_y'] = scale_y
        return result


class Tracker(object):
    def __init__(self, max_frame):
        self.flow_inferencer = Inferencer(task='flow', inference_size=(320, 640))
        self.super_point = SuperMatch(dst_size=(960, 640), nms_radius=5)
        self.keypoint_id_generator = IDGenerator()
        self.frame_id = -1
        self.frames = []
        self.keypoint_mask_rad = 20
        self.max_cz = 50
        self.max_cx = 5
        self.cx_range = [-10, 10]
        self.desc_match_th = 0.7
        self.min_kpt_num = 200
        self.tune_kpt = True
        self.tune_search_rad = 5
        self.last_data = None
        self.max_frame = max_frame
        self.orb = False

    def generate_keypoints(self, frame_id, image, mask):
        mask = max_filter(mask, 10)
        if self.orb:
            orb = cv2.ORB_create()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            kp = orb.detect(gray, None)
            points, des = orb.compute(gray, kp)
            results = []
            for idx, kpt in enumerate(points):
                x = int(kpt.pt[0])
                y = int(kpt.pt[1])
                desc = des[idx]
                if mask[y, x] == 0:
                    keypoint = KeyPoint()
                    keypoint.id = self.keypoint_id_generator.generate()
                    keypoint.frame_id = frame_id
                    keypoint.pi = np.array((x, y))
                    keypoint.desc = desc
                    results.append(keypoint)
            return results, None

        super_points_data = self.super_point.extract(image)

        points = super_points_data['keypoints'][0].detach().cpu().numpy()
        descriptors = super_points_data['descriptors'][0].detach().cpu().numpy()
        results = []
        for idx, (x, y) in enumerate(points):
            x = int(x)
            y = int(y)
            desc = descriptors[:, idx]
            if 0 < x < mask.shape[1] and 0 < y < mask.shape[0] and mask[y, x] == 0:
                keypoint = KeyPoint()
                keypoint.id = self.keypoint_id_generator.generate()
                keypoint.frame_id = frame_id
                keypoint.pi = np.array((x, y))
                keypoint.desc = desc
                results.append(keypoint)
        return results, super_points_data

    def generate_keypoint_mask(self, mask, keypoints, rad=3):
        h, w = mask.shape[:2]
        for keypoint in keypoints:
            x, y = int(keypoint.pi[0]), int(keypoint.pi[1])
            if 0 < x < w and 0 < y < h:
                mask[y, x] = 1
        mask = max_filter(mask, rad)
        return mask

    def generate_keypoint_idx_mat(self, mask, keypoints):
        mask.fill(-1)
        h, w = mask.shape[:2]
        for idx, keypoint in enumerate(keypoints):
            x, y = int(keypoint.pi[0]), int(keypoint.pi[1])
            if 0 < x < w and 0 < y < h:
                mask[y, x] = idx
        return mask

    def search_closest_kpt(self, x, y, kpt_idx_mat):
        min_x = max(x - self.tune_search_rad, 0)
        max_x = min(x + self.tune_search_rad, kpt_idx_mat.shape[1] - 1)
        min_y = max(y - self.tune_search_rad, 0)
        max_y = min(y + self.tune_search_rad, kpt_idx_mat.shape[0] - 1)
        dst_idx = -1
        best_distance = 10000000
        for r in range(min_y, max_y):
            for c in range(min_x, max_x):
                kpt_idx = kpt_idx_mat[r, c]
                if kpt_idx >= 0:
                    distance = (r - y) ** 2 + (c - x) ** 2
                    if distance < best_distance:
                        dst_idx = int(kpt_idx)
                        best_distance = distance
        return dst_idx


class StereoTracker(Tracker):
    def __init__(self, stereo_cache_root, stereo_cache_save_step=1, max_frame=None):
        super(StereoTracker, self).__init__(max_frame)
        self.stereo_inferencer = Inferencer(task='stereo', inference_size=(256, 480))
        self.stereo_cache_root = stereo_cache_root
        self.stereo_cache_save_step = stereo_cache_save_step
        self.check_with_imu_pose = True

    def init_cam(self, p1, p2, cam_to_imu, q_matrix, imu_height):
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.cam_to_imu = np.array(cam_to_imu)
        self.imu_to_cam = np.linalg.inv(self.cam_to_imu)
        self.imu_height = imu_height
        self.q_matrix = np.array(q_matrix)
        self.dis_flow = DisFlow(self.p1, self.imu_to_cam)

    def generate_stereo_3d(self, left, right):
        disparity, disparity_resize = self.stereo_inferencer.forward(left, right)
        points_3d_mat = cv2.reprojectImageTo3D(disparity.astype('float32'), self.q_matrix)
        return points_3d_mat, disparity, disparity_resize

    def vis(self, data, last_keypoints_dict, next_keypoints_dict, flow_im=None):
        image_concater = ImageConcater((360, 640))
        last_im = self.last_data['left'].copy()
        this_im = data['left'].copy()
        right = self.last_data['right'].copy()
        for k, v in last_keypoints_dict.items():
            if k in next_keypoints_dict:
                nv = next_keypoints_dict[k]
                lx, ly = int(v.pi[0]), int(v.pi[1])
                rx, ry = int(v.pr[0]), int(v.pr[1])
                nx, ny = int(nv.pi[0]), int(nv.pi[1])
                cv2.circle(last_im, (lx, ly), 3, (0, 0, 255), -1)
                cv2.circle(this_im, (nx, ny), 3, (0, 255, 0), -1)
                cv2.circle(right, (rx, ry), 3, (0, 255, 0), -1)

        image_concater.add_image(last_im, 0, 0, keep_ratio=False)
        image_concater.add_image(this_im, 1, 0, keep_ratio=False)
        image_concater.add_image(right, 0, 1, keep_ratio=False)
        if flow_im is not None:
            image_concater.add_image(flow_im, 1, 1, keep_ratio=False)
        render_result = image_concater.render()
        return render_result

    def get_next_frame_id(self):
        return self.frame_id + 1

    def track(self, data_dict, vis=False, frame_cache=None):
        self.frame_id += 1
        left = data_dict['left']
        right = data_dict['right']
        pose = data_dict['pose']
        mask = data_dict['mask']
        timestamp = data_dict['timestamp']

        if frame_cache is not None:
            self.last_data = data_dict
            self.frames.append(frame_cache)

        new_keypoints, super_points_data = self.generate_keypoints(self.frame_id, left, mask)
        keypoint_mask = self.generate_keypoint_mask(np.zeros(left.shape[:2]), new_keypoints, self.keypoint_mask_rad)
        keypoint_idx_mat = self.generate_keypoint_idx_mat(np.zeros(left.shape[:2]), new_keypoints)
        frame = Frame()
        frame.id = self.frame_id
        frame.odom_pose = pose
        frame.timestamp = timestamp
        points_3d_mat, disparity, disparity_resize = self.generate_stereo_3d(left, right)
        render_result = None

        if self.stereo_cache_root and self.frame_id % self.stereo_cache_save_step == 0:
            im_save_path = os.path.join(self.stereo_cache_root, "{}.jpg".format(frame.id))
            mat_save_path = os.path.join(self.stereo_cache_root, "{}.npz".format(frame.id))
            cv2.imwrite(im_save_path, left)
            np.savez(mat_save_path, q=self.q_matrix, disp=disparity_resize.astype('float32'))
            # np.save(mat_save_path, points_3d_mat)

        if len(self.frames) == 0:
            keypoint_dict = {}
            for keypoint in new_keypoints:
                x, y = int(keypoint.pi[0]), int(keypoint.pi[1])
                point_3d = points_3d_mat[y, x]
                if self.cx_range[0] < point_3d[0] < self.cx_range[1] and point_3d[2] < self.max_cz:
                # if point_3d[2] < self.max_cz:
                    keypoint.pc = point_3d
                    keypoint.pr = keypoint.pi.copy()
                    keypoint.pr[0] -= disparity[y, x]
                    keypoint_dict[keypoint.id] = keypoint
            frame.keypoints = keypoint_dict
            frame.odom_cam_pose = self.imu_to_cam.dot(matrix_utils.matrix_from_pose(pose)).dot(self.cam_to_imu)
        else:
            last_frame = self.frames[-1]
            delta_imu_pose = matrix_utils.relative_pose(frame.odom_pose, last_frame.odom_pose)
            delta_imu_matrix = matrix_utils.matrix_from_pose(delta_imu_pose)
            delta_cam_matrix = np.matmul(delta_imu_matrix, self.cam_to_imu)
            delta_cam_matrix = np.matmul(self.imu_to_cam, delta_cam_matrix)
            frame.odom_cam_pose = self.imu_to_cam.dot(matrix_utils.matrix_from_pose(pose)).dot(self.cam_to_imu)
            # frame.odom_cam_pose = self.imu_to_cam.dot(matrix_utils.matrix_from_pose(pose))
            last_keypoints_dict = last_frame.keypoints

            # using optical flow to predict the image point
            # flow = self.flow_inferencer.forward(self.last_data['left'], left)

            flow = self.dis_flow.forward(self.last_data['left'], left, self.last_data['pose'], pose)

            next_key_points_list = []
            for keypoint in last_keypoints_dict.values():
                lx, ly = int(keypoint.pi[0]), int(keypoint.pi[1])
                dx, dy = flow[ly, lx]
                nx, ny = lx + dx, ly + dy
                last_point_3d = keypoint.pc
                dst_kpt = None
                if self.tune_kpt:
                    dst_idx = self.search_closest_kpt(int(nx), int(ny), keypoint_idx_mat)
                    if dst_idx >= 0:
                        dst_kpt = new_keypoints[dst_idx]
                        nx, ny = dst_kpt.pi
                    else:
                        continue

                # drop out-of-image points
                if nx <= 0 or nx > left.shape[1]-1 or ny <= 0 or ny > left.shape[0]-1 \
                   or keypoint_mask[int(ny), int(nx)] == 0: # also drop points which are out of superpoint mask
                    continue

                next_point_flow_2d = np.array([nx, ny])
                next_point_flow_3d = points_3d_mat[int(ny), int(nx)]

                if next_point_flow_3d[0] > self.cx_range[1] or next_point_flow_3d[0] < self.cx_range[0] or next_point_flow_3d[2] > self.max_cz:
                    continue

                # compare distance between imu odom
                if self.check_with_imu_pose:
                    next_point_predict_3d = matrix_utils.homo_mul(last_point_3d[None, ...], delta_cam_matrix, False)[0]
                    if abs(next_point_predict_3d[0] - next_point_flow_3d[0]) > 0.5 or \
                       abs(next_point_predict_3d[1] - next_point_flow_3d[1]) > 0.5 or \
                       abs(next_point_predict_3d[2] - next_point_flow_3d[2]) > 0.5:
                        continue

                next_keypoint = KeyPoint()
                next_keypoint.id = keypoint.id
                next_keypoint.pi = next_point_flow_2d
                next_keypoint.pc = next_point_flow_3d
                next_keypoint.pr = next_keypoint.pi.copy()
                next_keypoint.pr[0] -= disparity[int(ny), int(nx)]
                if dst_kpt is not None:
                    next_keypoint.desc = dst_kpt.desc
                next_key_points_list.append(next_keypoint)

            next_keypoints_dict = {}

            if self.tune_kpt:
                for kpt in next_key_points_list:
                    match_score = np.sum(last_keypoints_dict[kpt.id].desc * kpt.desc)
                    next_keypoints_dict[kpt.id] = kpt
                    if match_score > self.desc_match_th:
                        next_keypoints_dict[kpt.id] = kpt

            else:
                # check whether the feature desc is match
                next_point_2d_list = []
                for kpt in next_key_points_list:
                    next_point_2d_list.append(kpt.pi)

                if len(next_point_2d_list) == 0:
                    print("debug: ", len(last_keypoints_dict))

                next_point_2d_tensor = torch.from_numpy(np.array(next_point_2d_list)).cuda().float()
                next_point_2d_tensor[:, 0] /= super_points_data['scale_x']
                next_point_2d_tensor[:, 1] /= super_points_data['scale_y']
                next_point_2d_descs = sample_descriptors(next_point_2d_tensor[None],
                                                         super_points_data['full_desc'], 8)[0].detach().cpu().numpy()
                for idx, kpt in enumerate(next_key_points_list):
                    last_desc = last_keypoints_dict[kpt.id].desc
                    next_desc = next_point_2d_descs[:, idx]
                    match_score = np.sum(last_desc*next_desc)
                    if match_score > self.desc_match_th:
                        kpt.desc = next_desc
                        next_keypoints_dict[kpt.id] = kpt

            if len(next_keypoints_dict) < self.min_kpt_num:
                next_keypoint_mask = self.generate_keypoint_mask(np.zeros(left.shape[:2]),
                                                                 next_keypoints_dict.values(), self.keypoint_mask_rad)
                for keypoint in new_keypoints:
                    x, y = int(keypoint.pi[0]), int(keypoint.pi[1])
                    point_3d = points_3d_mat[y, x]
                    if self.cx_range[0] < point_3d[0] < self.cx_range[1] and point_3d[2] < self.max_cz and next_keypoint_mask[y, x] == 0:
                    # if point_3d[2] < self.max_cz and next_keypoint_mask[y, x] == 0:
                        keypoint.pc = point_3d
                        keypoint.pr = keypoint.pi.copy()
                        keypoint.pr[0] -= disparity[int(y), int(x)]
                        next_keypoints_dict[keypoint.id] = keypoint
            frame.keypoints = next_keypoints_dict

            if vis:
                render_result = self.vis(data_dict, last_keypoints_dict, next_keypoints_dict, draw_flow(left, flow, 32))
                # cv2.imshow("", render_result)
                # cv2.waitKey()
        self.last_data = data_dict
        self.frames.append(frame)
        return render_result


class StereoRefiner(StereoTracker):
    def __init__(self, calibrator, *args, **kwargs):
        super(StereoRefiner, self).__init__(*args, **kwargs)
        self.frame_id_generator = IDGenerator()
        self.stereo_inferencer.inference_size = (256, 480)
        self.super_point = SuperMatch(dst_size=(960, 640), nms_radius=10)
        self.calibrator = calibrator
        self.matches_list = []
        self.new_rt = None

    def init_cam(self, p1, p2, cam_to_imu, q_matrix, imu_height):
        super(StereoRefiner, self).init_cam(p1, p2, cam_to_imu, q_matrix, imu_height)

    def vis_matches(self, data, matches):
        image_concater = ImageConcater((540, 960))
        left = data['left'].copy()
        right = data['right'].copy()
        for kpt_left, kpt_right in matches:
            lx, ly = int(kpt_left.pi[0]), int(kpt_left.pi[1])
            rx, ry = int(kpt_right.pi[0]), int(kpt_right.pi[1])
            cv2.circle(left, (lx, ly), 3, (0, 0, 255))
            cv2.circle(right, (rx, ry), 3, (0, 255, 0))

        image_concater.add_image(left, 0, 0, keep_ratio=False)
        image_concater.add_image(right, 0, 1, keep_ratio=False)
        render_result = image_concater.render()
        return render_result

    def points_to_mono_undistort(self, points, left=True):
        distort_points = self.calibrator.undistort_points(points, 'stereo', 0 if left else 1, reverse=True)
        if left:
            M, D, R, P = [self.calibrator.calib_params_dict['stereo'][k] for k in ['M1', 'D1', 'R1', 'P1']]
        else:
            M, D, R, P = [self.calibrator.calib_params_dict['stereo'][k] for k in ['M2', 'D2', 'R2', 'P2']]
        mono_undistort_points = cv2.undistortPoints(distort_points, M, D, None, M)

        # cv2.getOptimalNewCameraMatrix()
        return mono_undistort_points[:, 0, :]

    def triangle(self, left_points, right_points):
        m1 = np.array(self.calibrator.calib_params_dict['stereo']['M1']).dot(np.eye(3, 4))
        m2 = np.array(self.calibrator.calib_params_dict['stereo']['M2']).dot(np.eye(3, 4))
        R = np.array(self.calibrator.calib_params_dict['stereo']['R'])
        T = np.array(self.calibrator.calib_params_dict['stereo']['T'])
        RT = np.eye(4)
        RT[:3, :3] = R
        RT[:3, 3:] = T
        m2 = np.matmul(m2, RT)
        left_points = np.ascontiguousarray(left_points.transpose((1, 0)))
        right_points = np.ascontiguousarray(right_points.transpose((1, 0)))
        points_3d = cv2.triangulatePoints(m1,
                                          m2,
                                          left_points,
                                          right_points)
        points_3d = points_3d[:3] / points_3d[3:]
        return points_3d.transpose((1, 0))

    def track(self, data_dict, vis=False, frame_cache=None):
        self.frame_id += 1
        left = data_dict['left']
        right = data_dict['right']
        pose = data_dict['pose']
        left_keypoints, left_super_point_data = self.generate_keypoints(self.frame_id, left, np.zeros(left.shape[:2]))
        right_keypoints, right_super_point_data = self.generate_keypoints(self.frame_id, right, np.zeros(left.shape[:2]))

        right_keypoint_idx_mat = self.generate_keypoint_idx_mat(np.zeros(left.shape[:2]), right_keypoints)
        points_3d_mat, disparity, disparity_resize = self.generate_stereo_3d(left, right)

        matches = []

        with TimerWith('match'):
            for left_kpt in left_keypoints:
                lx, ly = int(left_kpt.pi[0]), int(left_kpt.pi[1])
                d = disparity[ly, lx]
                px, py = lx - d, ly
                dst_kpt = None
                dst_idx = self.search_closest_kpt(int(px), int(py), right_keypoint_idx_mat)
                if dst_idx >= 0:
                    dst_kpt = right_keypoints[dst_idx]

                if dst_kpt is not None and not dst_kpt.matched:
                    match_score = np.sum(left_kpt.desc*dst_kpt.desc)
                    if match_score > 0.8:
                        dst_kpt.matched = True
                        matches.append([left_kpt, dst_kpt])

        def extract_kpt_points(keypoints):
            result = []
            for kpt in keypoints:
                result.append(kpt.pi)
            return np.array(result)

        left_points = extract_kpt_points([m[0] for m in matches])
        right_points = extract_kpt_points([m[1] for m in matches])

        left_mono_undistort_points = self.points_to_mono_undistort(left_points, True)
        right_mono_undistort_points = self.points_to_mono_undistort(right_points, False)
        points_3d = self.triangle(left_mono_undistort_points, right_mono_undistort_points)
        for idx in range(len(matches)):
            matches[idx][0].pi = left_mono_undistort_points[idx]
            matches[idx][0].pc = points_3d[idx]
            matches[idx][1].pi = right_mono_undistort_points[idx]

        self.matches_list.append(matches)

        if vis:
            render_result = self.vis_matches(data_dict, matches)
            cv2.imshow("", render_result)
            cv2.waitKey(10)

    def generate_new_calib(self):
        m1 = np.array(self.calibrator.calib_params_dict['stereo']['M1'])
        m2 = np.array(self.calibrator.calib_params_dict['stereo']['M2'])
        d1 = np.array(self.calibrator.calib_params_dict['stereo']['D1'])
        d2 = np.array(self.calibrator.calib_params_dict['stereo']['D2'])
        h = self.calibrator.calib_params_dict['stereo']['height']
        w = self.calibrator.calib_params_dict['stereo']['width']
        R = self.new_rt[:3, :3]
        T = self.new_rt[:3, 3:]
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(m1, d1, m2, d2, (w, h), R, T)

        return {
            "M1": m1,
            "M2": m2,
            "D1": d1,
            "D2": d2,
            "height": h,
            "width": w,
            "rotate": self.calibrator.calib_params_dict['stereo']['rotate'],
            "R": R,
            "T": T,
            "R1": R1,
            "R2": R2,
            "P1": P1,
            "P2": P2,
            "Q": Q,
            "Tr_cam_to_imu": self.calibrator.calib_params_dict['stereo']['Tr_cam_to_imu'],
            "type": self.calibrator.calib_params_dict['stereo']['type'],
            "car": self.calibrator.calib_params_dict['stereo']['car'],
            "sensor_name": self.calibrator.calib_params_dict['stereo']['sensor_name'],
        }

    def optimize(self, iter=10):
        from visual_odom.optimize.pose_optimizer import BundleAdjust
        from plus_general.utils.matrix_utils import pose_from_matrix
        optimizer = BundleAdjust()
        optimizer.add_pose(0, np.identity(4), fix=True)
        m1 = np.array(self.calibrator.calib_params_dict['stereo']['M1']).dot(np.eye(3, 4))
        m2 = np.array(self.calibrator.calib_params_dict['stereo']['M2']).dot(np.eye(3, 4))
        R = np.array(self.calibrator.calib_params_dict['stereo']['R'])
        T = np.array(self.calibrator.calib_params_dict['stereo']['T'])
        RT = np.eye(4)
        RT[:3, :3] = R
        RT[:3, 3:] = T
        optimizer.add_pose(1, RT, fix=False)

        for matches in self.matches_list:
            for match in matches:
                optimizer.add_point_3d(match[0].id+2, match[0].pc, False)
                optimizer.add_edge(match[0].id+2, 0, match[0].pi, m1)
                optimizer.add_edge(match[0].id+2, 1, match[1].pi, m2)

        optimizer.optimize(iter, verbose=False)
        new_rt = optimizer.optimizer.vertex(1).estimate().matrix()
        # print("old", pose_from_matrix(RT))
        # print("new", pose_from_matrix(new_rt))
        p0 = np.array(pose_from_matrix(RT))
        p1 = np.array(pose_from_matrix(new_rt))
        diff = p1 - p0
        diff = diff[3:] * 180 /np.pi
        print(diff)
        self.new_rt = new_rt


