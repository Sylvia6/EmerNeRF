import g2o
import numpy as np
import math


class BundleAdjust(object):
    def __init__(self, robust_kernel=False):
        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
        # solver = g2o.BlockSolverSE3(g2o.LinearSolverDenseSE3())
        # solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
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
        return edge

    # def add_stereo_edge(self, point_idx, pose_idx, point_2d, p1, bf, Xw=None):
    #     edge = g2o.EdgeStereoSE3ProjectXYZOnlyPose()
    #     edge.set_vertex(0, self.optimizer.vertex(point_idx))
    #     edge.set_vertex(1, self.optimizer.vertex(pose_idx))
    #     edge.set_measurement(np.array(point_2d).astype(float))
    #     edge.set_information(np.identity(2))
    #     if self.robust_kernel:
    #         edge.set_robust_kernel(g2o.RobustKernelHuber(math.sqrt(5.991)))
    #     edge.fx = p1[0, 0]
    #     edge.fy = p1[1, 1]
    #     edge.cx = p1[0, 2]
    #     edge.cy = p1[1, 2]
    #     edge.bf = bf
    #     edge.Xw = pass
    #     self.optimizer.add_edge(edge)

    def optimize(self, max_iters=10, verbose=False):
        self.optimizer.initialize_optimization()
        if verbose:
            self.optimizer.set_verbose(verbose)
        self.optimizer.optimize(max_iters)


class PoseOnlyBundleAdjust(BundleAdjust):
    def set_pose(self, idx, matrix, fix):
        pose = g2o.SE3Quat(matrix[:3, :3], matrix[:3, 3].tolist())
        self.optimizer.vertex(idx).set_estimate(pose)
        self.optimizer.vertex(idx).set_fixed(fix)

    def add_mono_edge(self, point_idx, point_2d, p1, Xw):
        edge = g2o.EdgeSE3ProjectXYZOnlyPose()
        edge.set_vertex(0, self.optimizer.vertex(point_idx))
        edge.set_measurement(np.array(point_2d).astype(float))
        edge.set_information(np.identity(2))
        if self.robust_kernel:
            edge.set_robust_kernel(g2o.RobustKernelHuber(math.sqrt(7.815)))
        edge.pCamera = p1[0, 0]
        edge.Xw = np.array(Xw).astype(float)
        self.optimizer.add_edge(edge)
        return edge

    def add_stereo_edge(self, point_idx, point_2d, p1, bf, Xw, ur):
        edge = g2o.EdgeStereoSE3ProjectXYZOnlyPose()
        edge.set_vertex(0, self.optimizer.vertex(point_idx))
        edge.set_measurement(np.array([point_2d[0], point_2d[1], ur]).astype(float))
        edge.set_information(np.identity(3))
        if self.robust_kernel:
            edge.set_robust_kernel(g2o.RobustKernelHuber(math.sqrt(5.991)))
        edge.fx = p1[0, 0]
        edge.fy = p1[1, 1]
        edge.cx = p1[0, 2]
        edge.cy = p1[1, 2]
        edge.bf = bf
        edge.Xw = np.array(Xw).astype(float)
        self.optimizer.add_edge(edge)
        return edge