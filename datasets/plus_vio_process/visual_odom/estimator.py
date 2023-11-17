import random
import numpy as np
import g2o
import tqdm
import math
import sys
# import drive_python.perception.matrix_utils as matrix_utils
import plus_general.utils.matrix_utils as matrix_utils
from visual_odom.optimize.pose_optimizer import pose_optimization
# from vio.optimize.pose_optimizer import pose_optimization


def lmeds_3d(pts1, pts2, max_iters, threshold, choose_num=6):
    if len(pts1) < choose_num:
        return None, None
    pt_num = pts1.shape[0]
    idx_list = list(range(pt_num))

    pts1 = np.concatenate((pts1.transpose((1, 0)), np.ones((1, pt_num))))
    pts2 = np.concatenate((pts2.transpose((1, 0)), np.ones((1, pt_num))))
    best_inlier_num = 0
    best_inliers = None
    best_loss = 10000
    best_full = None
    best_distance = None
    for i in range(max_iters):
        idxs = random.sample(idx_list, choose_num)
        sample_1 = pts1[:-1, idxs]
        sample_2 = pts2[:-1, idxs]
        r, t = matrix_utils.rigid_transform_3D(sample_1, sample_2)
        full = np.eye(4)
        full[:3, :3] = r
        full[:3, -1] = t[:, 0]
        transform_pts = np.matmul(full, pts1)
        distance = np.sum((transform_pts-pts2)**2, 0)
        loss = np.median(distance)
        if loss < best_loss:
            best_full = full
            best_loss = loss
            best_distance = distance
    best_inliers = np.sqrt(best_distance) < threshold
    # if best_full is not None:
    #     sample_1 = pts1[:-1, best_inliers]
    #     sample_2 = pts2[:-1, best_inliers]
    #     r, t = rigid_transform_3D(sample_1, sample_2)
    #     full = np.eye(4)
    #     full[:3, :3] = r
    #     full[:3, -1] = t[:, 0]
    #     best_full = full
    return best_full, best_inliers


def ransac_3d(pts1, pts2, max_iters, threshold, choose_num=6):
    # choose_num = 6
    pt_num = pts1.shape[0]
    idx_list = list(range(pt_num))

    pts1 = np.concatenate((pts1.transpose((1, 0)), np.ones((1, pt_num))))
    pts2 = np.concatenate((pts2.transpose((1, 0)), np.ones((1, pt_num))))
    best_inlier_num = 0
    best_inliers = None
    best_loss = 10000
    best_full = None
    for i in range(max_iters):
        idxs = random.sample(idx_list, choose_num)
        # sample_1 = pts1[:-1, idxs].transpose((1, 0))
        # sample_2 = pts1[:-1, idxs].transpose((1, 0))
        sample_1 = pts1[:-1, idxs]
        sample_2 = pts2[:-1, idxs]
        r, t = matrix_utils.rigid_transform_3D(sample_1, sample_2)
        full = np.eye(4)
        full[:3, :3] = r
        full[:3, -1] = t[:, 0]
        # full = umeyama(sample_1.transpose(), sample_2.transpose(), False)
        transform_pts = np.matmul(full, pts1)
        distance = np.sqrt(np.sum((transform_pts-pts2)**2, 0))
        inliers = distance < threshold
        inlier_num = np.sum(inliers)
        if inlier_num > 0 :
            loss = np.mean(distance[inliers])
            # loss = np.median
            if inlier_num > best_inlier_num:
                best_inlier_num = inlier_num
                best_full = full
                best_loss = loss
                best_inliers = inliers
            elif inlier_num == best_inlier_num:
                if loss < best_loss:
                    best_inlier_num = inlier_num
                    best_full = full
                    best_loss = loss
                    best_inliers = inliers
    if best_full is not None:
        sample_1 = pts1[:-1, best_inliers]
        sample_2 = pts2[:-1, best_inliers]
        r, t = matrix_utils.rigid_transform_3D(sample_1, sample_2)
        full = np.eye(4)
        full[:3, :3] = r
        full[:3, -1] = t[:, 0]
        # full = umeyama(sample_1.transpose(), sample_2.transpose(), False)
        best_full = full
    return best_full, best_inliers


def least_square_3d(pts1, pts2):
    pt_num = pts1.shape[0]
    pts1 = np.concatenate((pts1.transpose((1, 0)), np.ones((1, pt_num))))
    pts2 = np.concatenate((pts2.transpose((1, 0)), np.ones((1, pt_num))))
    r, t = matrix_utils.rigid_transform_3D(pts1[:-1], pts2[:-1])
    full = np.eye(4)
    full[:3, :3] = r
    full[:3, -1] = t[:, 0]
    return full, np.ones(pt_num)


class BundleAdjust(object):
    def __init__(self, robust_kernel=False):
        optimizer = g2o.SparseOptimizer()
        # solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        optimizer.set_algorithm(solver)
        self.optimizer = optimizer
        self.robust_kernel = robust_kernel

    def clear(self):
        self.optimizer.clear()

    def add_pose(self, idx, matrix, fix):
        pose = g2o.SE3Quat(matrix[:3, :3], matrix[:3, 3].tolist())
        v_se3 = g2o.VertexSE3Expmap()
        v_se3.set_id(idx)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fix)
        self.optimizer.add_vertex(v_se3)

    def add_point_3d(self, idx, point_3d, fix=False):
        vp = g2o.VertexSBAPointXYZ()
        vp.set_id(idx)
        vp.set_marginalized(True)
        vp.set_estimate(point_3d)
        vp.set_fixed(fix)
        self.optimizer.add_vertex(vp)

    def add_edge(self, point_idx, pose_idx, point_2d, p1):
        edge = g2o.EdgeSE3ProjectXYZ()
        edge.set_vertex(0, self.optimizer.vertex(point_idx))
        edge.set_vertex(1, self.optimizer.vertex(pose_idx))
        edge.set_measurement(np.array(point_2d).astype(float))
        edge.set_information(np.identity(2))
        if self.robust_kernel:
            edge.set_robust_kernel(g2o.RobustKernelHuber(math.sqrt(5.991)))
        edge.fx = p1[0, 0]
        edge.fy = p1[1, 1]
        edge.cx = p1[0, 2]
        edge.cy = p1[1, 2]
        self.optimizer.add_edge(edge)

    def add_stereo_edge(self, point_idx, pose_idx, point_2d, p1, bf, ur):
        edge = g2o.EdgeStereoSE3ProjectXYZ()
        edge.set_vertex(0, self.optimizer.vertex(point_idx))
        edge.set_vertex(1, self.optimizer.vertex(pose_idx))
        edge.set_measurement(np.array([point_2d[0], point_2d[1], ur]).astype(float))
        edge.set_information(np.identity(3))
        if self.robust_kernel:
            edge.set_robust_kernel(g2o.RobustKernelHuber(math.sqrt(5.991)))
        edge.fx = p1[0, 0]
        edge.fy = p1[1, 1]
        edge.cx = p1[0, 2]
        edge.cy = p1[1, 2]
        edge.bf = bf
        self.optimizer.add_edge(edge)
        return edge

    def optimize(self, max_iters=10, verbose=False):
        self.optimizer.initialize_optimization()
        if verbose:
            self.optimizer.set_verbose(verbose)
        self.optimizer.optimize(max_iters)


class PoseEstimator(object):
    def __init__(self, frames, cam_params):
        self.frames = frames
        self.optimizer = BundleAdjust(robust_kernel=True)
        self.cam_params = cam_params
        self.p = self.cam_params['p']
        self.window_size = 10

    def add_frame(self, frame):
        self.frames.append(frame)
        if len(frame) == 1:
            self.frames[0].pose = np.identity(4)
        else:
            last_frame = self.frames[-2]
            curr_frame = self.frames[-1]
            kpt_match_pairs = []
            for id, kpt in last_frame.keypoints.items():
                if id in curr_frame.keypoints:
                    kpt_match_pairs.append([kpt, curr_frame.keypoints[id]])
            last_point_3d = []
            next_point_3d = []
            for kpt_pair in kpt_match_pairs:
                last_point_3d.append(kpt_pair[0].pc)
                next_point_3d.append(kpt_pair[1].pc)
            last_point_3d = np.array(last_point_3d)
            next_point_3d = np.array(next_point_3d)

            # matrix, inliers = lmeds_3d(last_point_3d, next_point_3d, 100, 0.5, 6)
            matrix, inliers = least_square_3d(last_point_3d, next_point_3d)
            if matrix is not None:
                # cam_pose = matrix.dot(last_frame.pose)
                relative_odom_cam_pose = matrix_utils.relative_matrix(curr_frame.odom_cam_pose, last_frame.odom_cam_pose)
                cam_pose = relative_odom_cam_pose.dot(last_frame.pose)
                curr_frame.pose = cam_pose
                curr_frame.points_to_global()
            else:
                relative_odom_cam_pose = matrix_utils.relative_matrix(curr_frame.odom_cam_pose, last_frame.odom_cam_pose)
                cam_pose = relative_odom_cam_pose.dot(last_frame.pose)
                curr_frame.pose = cam_pose
                curr_frame.points_to_global()

    def match_frame(self, idx_to_match):
        assert idx_to_match > 0
        last_frame = self.frames[idx_to_match - 1]
        curr_frame = self.frames[idx_to_match]
        kpt_match_pairs = []
        for id, kpt in last_frame.keypoints.items():
            if id in curr_frame.keypoints:
                if kpt.pc[2] > 10:
                    kpt_match_pairs.append([kpt, curr_frame.keypoints[id]])

        last_point_3d = []
        next_point_3d = []
        for kpt_pair in kpt_match_pairs:
            last_point_3d.append(kpt_pair[0].pc)
            next_point_3d.append(kpt_pair[1].pc)
        last_point_3d = np.array(last_point_3d)
        next_point_3d = np.array(next_point_3d)
        # matrix, inliers = lmeds_3d(last_point_3d, next_point_3d, 100, 0.5, 6)
        # if len(last_point_3d) > 10:
        #     matrix, inliers = ransac_3d(last_point_3d, next_point_3d, 100, 0.2, 6)
        #     # matrix, inliers = lmeds_3d(last_point_3d, next_point_3d, 100, 0.2, 6)
        # else:
        #     matrix, inliers = least_square_3d(last_point_3d, next_point_3d)
        # matrix, inliers = least_square_3d(last_point_3d, next_point_3d)
        matrix, inliers = 1, np.ones(len(last_point_3d))
        if matrix is not None:
            # if idx_to_match <= 2:
            # cam_pose = matrix.dot(last_frame.pose)
            # else:
            relative_odom_cam_pose = matrix_utils.relative_matrix(curr_frame.odom_cam_pose, last_frame.odom_cam_pose)
            cam_pose = relative_odom_cam_pose.dot(last_frame.pose)
            curr_frame.pose = cam_pose
            curr_frame.points_to_global()
            for pair_idx in range(len(inliers)):
                if not inliers[pair_idx]:
                    continue
                kpt_match_pairs[pair_idx][0].matched = True
                kpt_match_pairs[pair_idx][1].matched = True
        else:
            relative_odom_cam_pose = matrix_utils.relative_matrix(curr_frame.odom_cam_pose, last_frame.odom_cam_pose)
            cam_pose = relative_odom_cam_pose.dot(last_frame.pose)
            curr_frame.pose = cam_pose
            curr_frame.points_to_global()

    def estimate_period(self, start, end, match=True, optimize_step=10, fix_num=2):
        self.optimizer.clear()
        existed_3d_point_set = set()
        # add poses
        for frame_idx in range(start, end):
            if match:
                if frame_idx-start == 0:
                    self.frames[frame_idx].pose = np.linalg.inv(self.frames[frame_idx].odom_cam_pose)
                    self.frames[frame_idx].points_to_global()
                else:
                    self.match_frame(frame_idx)
            self.optimizer.add_pose(self.frames[frame_idx].id, self.frames[frame_idx].pose, frame_idx-start < fix_num)

        # add edge
        for frame_idx in range(start, end):
        # for frame in self.frames:
            for kpt_id, kpt in self.frames[frame_idx].keypoints.items():
                if kpt.matched:
                    if kpt_id not in existed_3d_point_set:
                        existed_3d_point_set.add(kpt_id)
                        self.optimizer.add_point_3d(kpt_id+len(self.frames), kpt.pg)
                    self.optimizer.add_edge(kpt_id+len(self.frames), self.frames[frame_idx].id, kpt.pi, self.p)
                    # self.optimizer.add_stereo_edge(kpt_id+len(self.frames), self.frames[frame_idx].id,
                    #                                kpt.pi, self.cam_params['p'], self.cam_params['bf'], kpt.pr[0])

        self.optimizer.optimize(optimize_step)

        # extract pose
        for frame_idx in range(start, end):
            self.frames[frame_idx].pose = self.optimizer.optimizer.vertex(self.frames[frame_idx].id).estimate().matrix()
            # self.frames[frame_idx].points_to_global()

        # extract point 3d
        existed_3d_point_dict = {}
        for point_id in existed_3d_point_set:
            new_point_3d = self.optimizer.optimizer.vertex(point_id+len(self.frames)).estimate()
            existed_3d_point_dict[point_id] = new_point_3d

        for frame_idx in range(start, end):
            for kpt_id, kpt in self.frames[frame_idx].keypoints.items():
                if kpt_id in existed_3d_point_dict:
                    kpt.pg = existed_3d_point_dict[kpt_id]

    def estimate_frame_by_frame(self, window_size=2):
        for frame_idx, frame in enumerate(tqdm.tqdm(self.frames)):
            if frame_idx == 0:
                self.frames[frame_idx].pose = np.identity(4)
                self.frames[frame_idx].points_to_global()
                continue
            self.match_frame(frame_idx)
            # self.frames[frame_idx].pose = np.identity(4)
            # self.frames[frame_idx].points_to_global()
            # self.frames[frame_idx].pose[0, 3] -= 1
            # self.frames[frame_idx].pose[1, 3] -= 1
            # self.frames[frame_idx].pose[2, 3] += 2
            # self.frames[frame_idx].points_to_global()
            # pose_optimization(self.frames[frame_idx-1:frame_idx+1], self.cam_params)

            # if window_size > 2:
            self.frames[frame_idx].points_to_global()
            start_idx = max(frame_idx - window_size, 0)
            pose_optimization(self.frames[start_idx:frame_idx + 1], self.cam_params)

            self.frames[frame_idx].points_to_global()

    def estimate_slide_window(self, optimize_step=5):
        for frame_idx in tqdm.tqdm(range(len(self.frames))):
            if frame_idx == 0:
                self.frames[frame_idx].pose = np.identity(4)
                self.optimizer.add_pose(frame_idx, np.identity(4), True)
                self.frames[frame_idx].points_to_global()
            else:
                self.match_frame(frame_idx)

            if frame_idx < 2:
                continue

            start = max(0, frame_idx-self.window_size)
            end = frame_idx+1

            # if end-start < self.window_size:
            #     fix_num = 2
            # else:
            #     fix_num = self.window_size // 2
            fix_num = 2
            self.estimate_period(start, end, match=False, fix_num=fix_num, optimize_step=optimize_step)

    def estimate(self, slide_window=False, optimize_step=10, fix_num=2):
        start = 0
        end = len(self.frames)
        if slide_window:
            self.estimate_slide_window(optimize_step)
            self.estimate_period(start, end, match=False, optimize_step=optimize_step, fix_num=fix_num)
        else:
            self.estimate_period(start, end, optimize_step=optimize_step, fix_num=fix_num)

    def get_result(self):
        return [[frame.id, frame.timestamp, np.linalg.inv(frame.pose)] for frame in self.frames]
        # return [[frame.id, frame.timestamp, frame.pose] for frame in self.frames]




