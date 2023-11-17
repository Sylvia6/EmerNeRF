import numpy as np
import g2o
from .bundle_adjust import BundleAdjust, PoseOnlyBundleAdjust


def pose_optimization(frames, cam_param):
    optimizer = PoseOnlyBundleAdjust(True)
    optimizer.add_pose(0, frames[-1].pose, False)
    edge_dict = {}
    outliers = {}
    # kpt_pg_dict = {}
    for kpt_id, kpt in frames[-1].keypoints.items():
        for frame in frames[:-1]:
            if kpt_id in frame.keypoints:
                dst_kpt = frame.keypoints[kpt_id]
                outliers[kpt_id] = False
                edge = optimizer.add_stereo_edge(0, kpt.pi, cam_param['p'], cam_param['bf'], dst_kpt.pg, kpt.pr[0])
                # kpt_pg_dict[kpt_id] = dst_kpt.pg
                edge_dict[kpt_id] = edge
                # break

    chi2_stereo = 7.815
    for it in range(4):
        optimizer.set_pose(0, frames[-1].pose, False)
        optimizer.optimize(10, False)

        for kpt_id in edge_dict.keys():
            edge = edge_dict[kpt_id]
            if outliers[kpt_id]:
                edge.compute_error()
            chi2 = edge.chi2()
            if chi2 > chi2_stereo:
                outliers[kpt_id] = True
                edge.set_level(1)
            else:
                outliers[kpt_id] = False
                edge.set_level(0)

            if it == 2:
                edge.remove_robust_kernel()

        if len(edge_dict) < 10:
            break
    pose = optimizer.optimizer.vertex(0).estimate().matrix()
    frames[-1].pose = pose


def track_with_motion(last_frame, frame):
    optimizer = PoseOnlyBundleAdjust()



