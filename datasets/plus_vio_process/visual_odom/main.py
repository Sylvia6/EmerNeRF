import copy
import json
import os
import torch
import cv2
import sys
import tqdm
import numpy as np
from plus_general.utils.bag_handler import BagHandler, ImageBagHandler
import plus_general.utils.matrix_utils as matrix_utils
from plus_general.utils.calibrator import Calibrator, SearchPolicy
from plus_general.utils.calibration_utils import load_opencv_yaml
from plus_general.utils.io_utils import get_file
from plus_general.utils.pcl_utils import write_color_pcd, write_pcd, write_type_pcd
from plus_general.utils.point_cloud2 import read_points
from visual_odom.utils import make_dir, extract_point_cloud, generate_obstacle_mask, extract_single_point_cloud
from visual_odom.core import Frame, load_frame, save_frame
import torch.nn.functional as F
from plus_general.csrc.common_proto_py.perception import obstacle_detection_pb2
from plus_general.csrc.common_proto_py.localization import localization_pb2
from visual_odom.utils import PathManager, get_stereo_calibrator
from plus_general.utils.config_utils2 import CfgNode
import pickle
import plus_general.utils.calibration_utils as calibration_utils

from scipy.spatial.transform import Rotation as R   

def save_pickle(obj, path):
    with open(path, "wb") as f:
        pickle.dump(obj, f)


def load_pickle(path):
    with open(path, "rb") as f:
        return pickle.load(f)


def get_video_writer(save_path):
    import skvideo.io
    video_in_dict = {
        
    }
    video_out_dict = {
        "-vb" : "5M",
        # "-c:v": "libx264",
        # "-pix_fmt": "yuv420p"
    }

    writer = skvideo.io.FFmpegWriter(save_path,
                                     inputdict=video_in_dict, outputdict=video_out_dict)
    return writer


def convert_boolstr(v):
    if v.lower() == "true":
        return True
    elif v.lower() == "false":
        return False
    else:
        raise TypeError("{} is not a valid bool string".format(v))


class PipelineCfg(CfgNode):
    def __init__(self):
        super(PipelineCfg, self).__init__()
        self.overwrite_track_cache = True
        self.bag_name = ""
        self.cache_pre = ""
        self.track_save_stride = 2
        self.max_track_frame = None
        self.first_pose_as_identity = False
        self.vis_track = False
        self.skip_cached_pipeline = False
        self.global_cloud = True
        self.global_cloud_spatial_stride = 3
        self.global_cloud_roi = [-15, 15, -5, 10, 20, 60]
        self.optimize_step = 10
        self.drive_root = None
        self.calib_db_root = None

        self.clear_cache = False
        self.check_skip_flag = False
        self.skip_flag = False
        self.use_obs_mask = True
        self.cameras = ["front_left", "front_right"]
        self.nerf = False

    def generate_aug(self):
        import argparse
        parser = argparse.ArgumentParser()
        for k, v in self.items():
            if isinstance(v, list):
                parser.add_argument("--{}".format(k), nargs="+", type=int, default=v)
            elif isinstance(v, bool):
                parser.add_argument("--{}".format(k), type=convert_boolstr, default=v)
            elif isinstance(v, int):
                parser.add_argument("--{}".format(k), type=int, default=v)
            else:
                parser.add_argument("--{}".format(k), default=v)
        return parser


def cache_pipeline(func):
    func_name = func.__name__
    def wrapper(*args, **kwargs):
        print("{}...".format(func_name))
        self = args[0]
        if self.cfg.skip_cached_pipeline and self.pipeline_cache.get(func_name, False) \
                and func_name not in self.non_skip_pipeline:
            return
        func(*args, **kwargs)
        self.pipeline_cache[func_name] = True
        self.save_pipeline_cache()
    return wrapper


class CalibrationFileManager(object):
    def __init__(self, calib_db_root):
        self.calib_db_root = calib_db_root
        calib_files = os.listdir(calib_db_root)
        self.calib_files = [file for file in calib_files if file.endswith(".yml")]

    def build_calibrator(self, bagname, calib_names, full_res=False):
        bagname = os.path.basename(bagname)
        car_name = bagname.split("_")[1]
        date = int(bagname.split("_")[0].split("T")[0])

        last_date_dict = {}
        dst_file_dict = {}
        for file in self.calib_files:
            if file.find(car_name) == -1:
                continue

            if "T" in file.split("_")[1]:
                file_calib_date = int(file.split("_")[1].split("T")[0])
            else:
                file_calib_date = int(file.split("_")[1])

            date_key_words = file.split("_")[1]

            for calib_name in calib_names:
                search_key = str(date_key_words) + "_" + calib_name
                if file.find(search_key) != -1:
                    if file_calib_date <= date:
                        if calib_name not in last_date_dict or last_date_dict[calib_name] < file_calib_date:
                            last_date_dict[calib_name] = file_calib_date
                            dst_file_dict[calib_name] = os.path.join(self.calib_db_root, file)

        mappping = {
            "front_left_right": "stereo",
            "rear_left_camera": "rear_left",
            "rear_right_camera": "rear_right",
            "side_left_camera": "side_left",
            "side_right_camera": "side_right",
        }

        calib_keys = []
        calib_params_dict = {}
        for calib_name in calib_names:
            dst_calib_name = calib_name
            if calib_name in mappping:
                dst_calib_name = mappping[calib_name]
            calib_keys.append(dst_calib_name)

            if dst_file_dict.get(calib_name, None) is None:
                print("not find {} for {}!".format(calib_name, bagname))

            data = load_opencv_yaml(dst_file_dict[calib_name])
            calib_params_dict[dst_calib_name] = data
        return Calibrator(calib_params_dict, calib_keys)


class Pipeline(object):
    def __init__(self, cfg):
        self.cfg = PipelineCfg().merge(cfg)
        self.path_manager = None
        self.calibrator = None
        self.pipeline_cache = {}
        self.non_skip_pipeline = []
        self.calibrator_manager = None

        if self.cfg.drive_root is not None:
            calibration_utils.drive_dir = self.cfg.drive_root
        if self.cfg.calib_db_root is not None:
            calibration_utils.calib_db_dir = self.cfg.calib_db_root

    def load_pipeline_cache(self):
        pipeline_cache_path = self.path_manager.join_root("pipeline_cache.json")
        if os.path.exists(pipeline_cache_path):
            with open(pipeline_cache_path) as f:
                data = json.load(f)
                self.pipeline_cache = data
        else:
            self.pipeline_cache = {}

    def save_pipeline_cache(self):
        pipeline_cache_path = self.path_manager.join_root("pipeline_cache.json")
        with open(pipeline_cache_path, "w") as f:
            json.dump(self.pipeline_cache, f)

    @cache_pipeline
    def track(self):
        mask_save_dir = self.path_manager.join_root("box_masks")
        cam_mask_dirs = [os.path.join(mask_save_dir, cam) for cam in self.cfg.cameras]
        for cam_mask_dir in cam_mask_dirs:
            os.makedirs(cam_mask_dir, exist_ok=True)

        from visual_odom.track.tracker import StereoTracker
        obstacle_topic = "/perception/obstacles"
        if self.cfg.use_obs_mask:
            other_topics = [obstacle_topic]
        else:
            other_topics = None
        bag_handler = ImageBagHandler(self.path_manager.bag_name, front_left=True, front_right=True,
                                      odom=True, other_topics=other_topics)
        if self.cfg.vis_track:
            video_writer = get_video_writer(self.path_manager.join_root("track_video.mp4"))
        tracker = StereoTracker(self.path_manager.stereo_cache_root, self.cfg.track_save_stride)
        tracker.max_cz = 200
        tracker.cx_range = [-10, 10]
        tracker.init_cam(
            self.calibrator.calib_params_dict["stereo"]["P1"],
            self.calibrator.calib_params_dict["stereo"]["P2"],
            self.calibrator.calib_params_dict["stereo"]["Tr_cam_to_imu"],
            self.calibrator.calib_params_dict["stereo"]["Q"],
            0.52
        )
        first_pose = None
        for idx, msg_dict in enumerate(tqdm.tqdm(bag_handler.msg_generator(to_image=True, process_odom=True))):
            if self.cfg.max_track_frame is not None and idx > self.cfg.max_track_frame:
                break
            left = msg_dict["front_left"].message
            left_unwarp = self.calibrator.unwarp(left, "stereo", 0)
            right = msg_dict["front_right"].message
            right_unwarp = self.calibrator.unwarp(right, "stereo", 1)
            left_image_timestamp = msg_dict["front_left"].timestamp.to_sec()
            imu_to_world, pose = msg_dict["imu_to_world"], msg_dict["pose"]

            if self.cfg.first_pose_as_identity:
                if first_pose is None:
                    first_pose = pose
                pose = matrix_utils.relative_pose(first_pose, pose)
            if self.cfg.use_obs_mask:
                pb = obstacle_detection_pb2.ObstacleDetection()
                pb.ParseFromString(msg_dict[obstacle_topic][0].message.data)
                mask = generate_obstacle_mask(left.shape[0], left.shape[1], pb, scale=1.3)
            else:
                mask = np.zeros(left_unwarp.shape[:2])
            if mask.shape[0] != left_unwarp.shape[0] or mask.shape[1] != left_unwarp.shape[1]:
                mask = cv2.resize(mask, (left_unwarp.shape[1], left_unwarp.shape[0]), cv2.INTER_NEAREST)
            
            # for streetsurf mask
            rgb_mask = mask * 255
            cv2.imwrite(os.path.join(mask_save_dir, "front_left", f"{idx:06d}.png"), rgb_mask)

            data = {
                "left": left_unwarp,
                "right": right_unwarp,
                "mask": mask,
                "pose": pose,
                "timestamp": left_image_timestamp
            }
            render_frame = tracker.track(data, self.cfg.vis_track)
            if render_frame is not None:
                video_writer.writeFrame(render_frame[:, :, ::-1])
            frame = tracker.frames[-1]
            save_path = os.path.join(self.path_manager.frame_root, "{}.pkl".format(frame.id))
            save_frame(frame, save_path)

        if self.cfg.first_pose_as_identity:
            save_pickle(first_pose, self.path_manager.join_root("first_pose.pkl"))
        
    @cache_pipeline
    def pose_estimate(self):
        from visual_odom.estimator import PoseEstimator
        cam_params = {
            "p": np.array(self.calibrator.calib_params_dict["stereo"]["P1"]),
            "b": np.linalg.norm(np.array(self.calibrator.calib_params_dict["stereo"]["T"])),
        }
        cam_params["bf"] = cam_params["p"][0, 0] * cam_params["b"]

        frame_files = get_file(self.path_manager.frame_root, [".pkl"])
        frames = []
        for file in frame_files:
            frame = load_frame(file)
            frames.append(frame)

        frames.sort(key=lambda x: x.id)
        if self.cfg.max_track_frame is not None:
            frames = frames[:self.cfg.max_track_frame]
        pose_estimator = PoseEstimator(frames, cam_params)
        pose_estimator.estimate(slide_window=True, optimize_step=self.cfg.optimize_step, fix_num=1)
        results = pose_estimator.get_result()
        save_pickle(results, self.path_manager.pose_path)

    @cache_pipeline
    def extract_global_cloud(self, poses=[], suffix=""):
        all_colors = []
        all_points = []
        if len(poses) <= 0:
            poses = load_pickle(self.path_manager.pose_path) # vio-pose
        else:
            poses = [[i, 0, pose] for i, pose in enumerate(poses)]

        # pose_results_dom = [
        #     load_pickle(os.path.join(self.path_manager.frame_root, frame))
        #     for frame in sorted(os.listdir(self.path_manager.frame_root))
        #     ]
        # for pose_idx, (id, timestamp, pose) in enumerate(tqdm.tqdm(pose_results_dom)):
        for pose_idx, (id, timestamp, pose) in enumerate(tqdm.tqdm(poses)):
            im_path = os.path.join(self.path_manager.stereo_cache_root, "{}.jpg".format(id))
            mat_path = os.path.join(self.path_manager.stereo_cache_root, "{}.npz".format(id))
            if not os.path.exists(mat_path):
                continue

            if self.cfg.max_track_frame is not None and id > self.cfg.max_track_frame:
                break

            image = cv2.imread(im_path)
            disp_data = np.load(mat_path)
            Q = disp_data["q"]
            disp = disp_data["disp"].astype("float32")
            scale = image.shape[1] / float(disp.shape[1])
            disp = F.interpolate(torch.from_numpy(disp)[None].unsqueeze(1), size=image.shape[:2],
                                 mode="bilinear",
                                 align_corners=True).squeeze(1)[0].numpy()
            disp *= scale
            mat_3d = cv2.reprojectImageTo3D(disp.astype("float32"), Q)
            dst_size = None
            points_3d, points_color = extract_single_point_cloud(image, mat_3d, pose=None, mask=None,
                                                                 dst_size=dst_size,
                                                                 spatial_stride=self.cfg.global_cloud_spatial_stride,
                                                                 roi=self.cfg.global_cloud_roi)
            points_3d = matrix_utils.homo_mul(points_3d, pose, image=False)
            if len(points_3d) > 0:
                all_colors.append(points_color)
                all_points.append(points_3d)
        all_points = np.concatenate(all_points, 0)
        all_colors = np.concatenate(all_colors, 0)
        if suffix == "":
            writepath = self.path_manager.join_root(f"global.pcd")
        else:
            writepath = self.path_manager.join_root(f"global_{suffix}.pcd")
        write_color_pcd(all_points, all_colors, writepath)

    def get_calibrator(self):
        calibrator = Calibrator.from_bag_message(self.path_manager.bag_name)
        if "stereo" not in calibrator.calib_params_dict:
            if self.calibrator_manager is None:
                self.calibrator_manager = CalibrationFileManager(self.cfg.calib_db_root)
            calib_names = ["front_left_right", "rear_left_camera", "rear_right_camera",
                           "side_left_camera", "side_right_camera"]
            calibrator = self.calibrator_manager.build_calibrator(self.path_manager.bag_name, calib_names)
        return calibrator

    def clear_cache(self):
        import shutil
        path_to_remove = [
            self.path_manager.stereo_cache_root,
            self.path_manager.frame_root
        ]
        for path in path_to_remove:
            if os.path.exists(path):
                shutil.rmtree(path)
        self.pipeline_cache["track"] = False
        self.save_pipeline_cache()

    def generate_nerf_data(self):
        print("Generating nerf data.")
        pose_results = load_pickle(self.path_manager.pose_path)
        pose_dict = {}
        for idx, ts, pose in pose_results:
            pose_dict[ts] = pose
        
        localization_topic = "/localization/state"
        lidar_topic = "/rslidar_points"
        other_topics = [localization_topic, lidar_topic]
        bag_handler = ImageBagHandler(self.path_manager.bag_name, front_left=True, front_right=True,
                                      odom=True, other_topics=other_topics)

        h, w = self.calibrator.calib_params_dict["stereo"]["height"], self.calibrator.calib_params_dict["stereo"]["width"]
        p1 = np.array(self.calibrator.calib_params_dict["stereo"]["P1"])
        p2 = np.array(self.calibrator.calib_params_dict["stereo"]["P2"])
        cam_t = p2[0, 3] / p2[0, 0]
        cam_t_pose = np.eye(4)
        cam_t_pose[0, 3] = cam_t

        image_save_dir = self.path_manager.join_root("images")
        cam_dirs = [os.path.join(image_save_dir, cam) for cam in self.cfg.cameras]
        for cam_dir in cam_dirs:
            os.makedirs(cam_dir, exist_ok=True)
          
        lc_to_imu = self.calibrator.calib_params_dict["stereo"]["Tr_cam_to_imu"]
        rc_to_imu = lc_to_imu @ np.linalg.inv(cam_t_pose)

        eqdcs = []
        vios = []
        for idx, msg_dict in enumerate(tqdm.tqdm(bag_handler.msg_generator(to_image=True))):
            left_image_timestamp = msg_dict["front_left"].timestamp.to_sec()
            if left_image_timestamp not in pose_dict:
                print(f"pass frame {idx}......")
                continue
            
            lc_to_w_vio = pose_dict[left_image_timestamp]   # 相对first_frame的pose
            vios.append(lc_to_w_vio)

            if localization_topic in msg_dict.keys():
                pb = localization_pb2.LocalizationEstimation()
                pb.ParseFromString(msg_dict[localization_topic][0].message.data)
                pos, quat = pb.position, pb.orientation
                ret = quat_xyz_to_matrix([quat.qx, quat.qy, quat.qz, quat.qw], [pos.x, pos.y, pos.z])
                eqdcs.append(ret @ lc_to_imu)
        
        eqdcs = np.array(eqdcs) 
        vios = np.array(vios)

        window_size = 10 // 2
        eqdc_fine = []

        # window_slide eqdc to make it smooth
        for i in range(len(eqdcs)):
            # window to i
            window_eqdcs = np.linalg.inv(eqdcs[i]) @ eqdcs[max(0, i - window_size) : min(len(eqdcs), i + window_size)]
            window_vios = np.linalg.inv(vios[i]) @ vios[max(0, i - window_size) : min(len(vios), i + window_size)]
            
            trans = np.array([np.linalg.inv(window_vios[j]) @ window_eqdcs[j] for j in range(len(window_eqdcs))])
            # trans = np.array([np.linalg.inv(window_eqdcs[j]) @ window_vios[j] for j in range(len(window_eqdcs))])
            mean_trans = average_se3_transformations(trans)
            eqdc_fine.append(eqdcs[i] @ mean_trans)

        eqdc_fine = np.array(eqdc_fine)
        imu_eqdc = eqdc_fine @ np.linalg.inv(np.asarray(lc_to_imu))

        # base_pose = eqdc_fine[0]
        # eqdc_to_0 = np.linalg.inv(base_pose) @ eqdc_fine
        # self.extract_global_cloud(poses=eqdc_fine, suffix="eqdc_vio")
        
        frames = []
        tmp_idx = -1    # 索引记录未丢帧的pose数据
        for idx, msg_dict in enumerate(tqdm.tqdm(bag_handler.msg_generator(to_image=True))):
            left_image_timestamp = msg_dict["front_left"].timestamp.to_sec()
            if left_image_timestamp not in pose_dict:
                print(f"pass frame {idx}......")
                tmp_idx -= 1
                continue
            else:
                tmp_idx += 1

            # lidar
            lidar_pts = None
            if lidar_topic in msg_dict:
                pc_msg = msg_dict[lidar_topic][0].message
                if lidar_topic == '/livox/lidar' or lidar_topic == '/rslidar_points' or lidar_topic == '/innovusion_lidar/iv_points':
                    lidar_pts = np.array(list(read_points(pc_msg, field_names=['x', 'y', 'z', 'intensity'], skip_nans=True)), dtype=np.float32)
                    lidar_pts[:,:3] = lidar_pts[:,:3] * 0.01
                else:
                    lidar_pts = np.array(list(read_points(pc_msg, field_names=['x', 'y', 'z', 'intensity', 'ring'], skip_nans=True)), dtype=np.float32)
            if lidar_pts is not None:
                lidar_pts.tofile(os.path.join(self.path_manager.cloud_root, f"{idx:06d}.bin"))

            # image 
            left = msg_dict["front_left"].message
            left_unwarp = self.calibrator.unwarp(left, "stereo", 0)
            right = msg_dict["front_right"].message
            right_unwarp = self.calibrator.unwarp(right, "stereo", 1)

            # pose 
            #       1. odom pose / eqdc pose
            # imu_to_w, pose = np.matrix(msg_dict["imu_to_world"]), msg_dict["pose"]    # odom
            imu_to_w = imu_eqdc[tmp_idx]    # eqdc
            lc_to_w = imu_to_w @ lc_to_imu
            rc_to_w = lc_to_w @ np.linalg.inv(cam_t_pose)

            #       2. vio pose
            lc_to_w_vio = pose_dict[left_image_timestamp]   # 相对first_frame的pose
            rc_to_w_vio = lc_to_w_vio @ np.linalg.inv(cam_t_pose)
            imu_to_w_vio = lc_to_w_vio @ np.linalg.inv(lc_to_imu)

            frames.append({
                "idx": "{:06d}".format(idx),
                "file_path": "images/front_left/{:06d}.png".format(idx),
                "transform_matrix": lc_to_w.tolist(), 
                "transform_matrix_vio": lc_to_w_vio.tolist(),
                "imu2w": imu_to_w.tolist(),
                "imu2w_vio": imu_to_w_vio.tolist(), 
                "c2imu": lc_to_imu.tolist(),
                "cam_name": "front_left",
                "intr": p1.tolist(),
                "timestamp": left_image_timestamp,
            })
            cv2.imwrite(os.path.join(image_save_dir, "front_left", "{:06d}.png".format(idx)), left_unwarp)

            frames.append({
                "idx": "{:06d}".format(idx),
                "file_path": "images/front_right/{:06d}.png".format(idx),
                "transform_matrix": rc_to_w.tolist(),
                "transform_matrix_vio": rc_to_w_vio.tolist(),
                "imu2w": imu_to_w.tolist(),
                "imu2w_vio": imu_to_w_vio.tolist(),
                "c2imu": rc_to_imu.tolist(),
                "cam_name": "front_right",
                "intr": p2.tolist(),
                "timestamp": left_image_timestamp,
            })       
            cv2.imwrite(os.path.join(image_save_dir, "front_right", "{:06d}.png".format(idx)), right_unwarp)   


        # 1. nerfstudio data / emernerf data
        # prepare for transforms.json data
        transforms_data = {
            "camera_model": "OPENCV", 
            "fl_x": p1[0, 0], # focal length x
            "fl_y": p1[1, 1], # focal length y
            "cx": p1[0, 2], # principal point x
            "cy": p1[1, 2], # principal point y
            "w": w, # image width
            "h": h, # image height
            "k1": 0, # first radial distorial parameter, used by [OPENCV, OPENCV_FISHEYE]
            "k2": 0, # second radial distorial parameter, used by [OPENCV, OPENCV_FISHEYE]
            "k3": 0, # third radial distorial parameter, used by [OPENCV_FISHEYE]
            "k4": 0, # fourth radial distorial parameter, used by [OPENCV_FISHEYE]
            "p1": 0, # first tangential distortion parameter, used by [OPENCV]
            "p2": 0, # second tangential distortion parameter, used by [OPENCV]
        }
        transforms_data["frames"] = frames
        transfrom_json_fpath = self.path_manager.join_root("transforms.json")
        with open(transfrom_json_fpath, "w") as f:
            json.dump(transforms_data, f, indent=2)
            print(f"=> transforms.json saved to {transfrom_json_fpath}")


        # 2. streetsurf data
        # prepare for scenario.pt data
        scene_objects = dict()
        scene_observers = dict()  
        n_frames = len(frames)//len(self.cfg.cameras)
        if "ego_car" not in scene_observers:
            scene_observers["ego_car"] = dict(
                class_name="EgoVehicle", n_frames=n_frames, 
                data=dict(v2w=[], timestamp=[], global_frame_ind=[]))
        for j in range(n_frames):
            frame = frames[::len(self.cfg.cameras)][j]
            scene_observers["ego_car"]["data"]["v2w"].append(frame["imu2w"])
            scene_observers["ego_car"]["data"]["timestamp"].append(frame["timestamp"])
            scene_observers["ego_car"]["data"]["global_frame_ind"].append(j)

        for i, cam_i in enumerate(self.cfg.cameras):
            if cam_i not in scene_observers:
                scene_observers[cam_i] = dict(
                    class_name="Camera", n_frames=n_frames, 
                    data=dict(hw=[], intr=[], distortion=[], c2v_0=[], c2v=[], sensor_v2w=[], c2w=[], timestamp=[], global_frame_ind=[]))
            for j in range(n_frames):
                frame = frames[j + i*len(self.cfg.cameras)]
                scene_observers[cam_i]["data"]["hw"].append((h, w))
                scene_observers[cam_i]["data"]["intr"].append(frame["intr"])
                scene_observers[cam_i]["data"]["c2w"].append(frame["transform_matrix"]) # ????
                # scene_observers[cam_i]["data"]["c2w"].append(frame["transform_matrix"] @ opencv2opengl) # streetsurf : opencv
                scene_observers[cam_i]["data"]["c2v"].append(frame["c2imu"])
                scene_observers[cam_i]["data"]["sensor_v2w"].append(frame["imu2w"])
                scene_observers[cam_i]["data"]["timestamp"].append(frame["timestamp"])
                scene_observers[cam_i]["data"]["global_frame_ind"].append(j)     
        
        for oid, odict in scene_observers.items():
            for k, v in odict["data"].items():
                odict["data"][k] = np.array(v)  
        for oid, odict in scene_objects.items():
            obj_annos = odict.pop("frame_annotations")   

        world_offset = np.zeros([3,])
        scene_metas = dict(world_offset=world_offset)
        scene_metas["n_frames"] = n_frames
        scene_metas["up_vec"] = "+z"

        scenario = dict()
        scenario["scene_id"] = os.path.basename(self.path_manager.bag_name).replace(".db","")
        scenario["metas"] = scene_metas
        scenario["objects"] = scene_objects
        scenario["observers"] = scene_observers
        scenario_fpath = self.path_manager.join_root("scenario.pt")
        with open(scenario_fpath, "wb") as f:
            pickle.dump(scenario, f)
            print(f"=> scenario.pt saved to {scenario_fpath}")

            
    def run(self, bag_name=None):
        if bag_name is None:
            bag_name = self.cfg.bag_name
        self.path_manager = PathManager(self.cfg.cache_pre, bag_name)
        self.load_pipeline_cache()
        if self.cfg.check_skip_flag:
            if self.pipeline_cache.get(self.cfg.skip_flag, False):
                return
        self.calibrator = self.get_calibrator()
        if not os.path.exists(self.path_manager.pose_path):
            self.track()
            self.pose_estimate()
        else:
            print(f"Tracked previously, using {self.path_manager.pose_path}!")

        if not os.path.exists(self.path_manager.join_root("global.pcd")):
            if self.cfg.global_cloud:
                self.extract_global_cloud()

        if self.cfg.nerf:
            self.generate_nerf_data()

        if self.cfg.clear_cache:
            self.clear_cache()


def quat_xyz_to_matrix(quat, translation):
    ret = np.eye(4)
    ret[:3,:3] = R.from_quat(quat).as_matrix()
    ret[:3,-1] = translation
    return ret

# 计算平均SE(3)变换
def average_se3_transformations(transformations):
    # 将SE(3)变换矩阵表示为四元数和平移向量
    def decompose_se3(T):
        R_matrix = T[:3, :3]
        translation = T[:3, 3]
        quat = R.from_matrix(R_matrix).as_quat()
        return quat, translation
    # 计算四元数的平均
    def average_quaternion(quaternions):
        average_quat = np.mean(quaternions, axis=0)
        average_quat /= np.linalg.norm(average_quat)
        return average_quat

    quaternions = []
    translations = []
    for T in transformations:
        quat, translation = decompose_se3(T)
        quaternions.append(quat)
        translations.append(translation)

    # 计算四元数的平均
    average_quat = average_quaternion(quaternions)

    # 计算平均平移向量
    average_translation = np.mean(translations, axis=0)

    # 构建平均SE(3)变换矩阵
    average_rotation_matrix = R.from_quat(average_quat).as_matrix()
    average_se3 = np.eye(4)
    average_se3[:3, :3] = average_rotation_matrix
    average_se3[:3, 3] = average_translation

    return average_se3

def cam_pose_to_nerf(cam_pose, gl=True):  
    """
        plus_data的camera为opencv坐标系
        
        emernerf的数据坐标输入为opencv坐标，nerfstudio的数据坐标输入为opengl坐标
        
        nerf_world与eqdc坐标系一样
        
        < opencv / colmap convention >                --->>>     < opengl / NeRF convention >                    --->>>   < world convention >
        facing [+z] direction, x right, y downwards   --->>>    facing [-z] direction, x right, y upwards        --->>>  facing [+x] direction, z upwards, y left
                    z                                              y ↑                                                      z ↑    x
                   ↗                                                 |                                                        |   ↗ 
                  /                                                  |                                                        |  /
                 /                                                   |                                                        | /
                o------> x                                           o------> x                                    y ←--------o
                |                                                   /
                |                                                  /
                |                                               z ↙
                ↓ 
                y
    """
    if gl:
        opencv2opengl = np.array([[1,0,0,0], [0,-1,0,0], [0,0,-1,0],[0,0,0,1]]).astype(float)
        opengl2opencv = np.linalg.inv(opencv2opengl)
        opengl2world = np.array([[0,0,-1,0], [-1,0,0,0], [0,1,0,0],[0,0,0,1]]).astype(float)
        gl_pose = opencv2opengl @ cam_pose @ opengl2opencv  
        world_pose = opengl2world @ gl_pose
    else:
        opencv2world = np.array([[0,0,1,0], [-1,0,0,0], [0,-1,0,0],[0,0,0,1]]).astype(float)
        world_pose = opencv2world @ cam_pose
    return world_pose

if __name__ == "__main__":
    cfg = PipelineCfg()
    parser = cfg.generate_aug()
    args = parser.parse_args()
    args = vars(args)
    cfg.merge(args)

    pipeline = Pipeline(cfg)
    # pipeline.non_skip_pipeline = ["pose_estimate"]
    pipeline.run()
