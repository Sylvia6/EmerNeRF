import logging
import os
from typing import Dict, List

import numpy as np
import torch
from omegaconf import OmegaConf
from torch import Tensor
from tqdm import trange
from PIL import Image

from torch.utils.data import ConcatDataset

from datasets.base.lidar_source import SceneLidarSource
from datasets.base.pixel_source import ScenePixelSource
from datasets.base.scene_dataset import SceneDataset
from datasets.base.split_wrapper import SplitWrapper
from datasets.utils import voxel_coords_to_world_coords
from radiance_fields.video_utils import depth_visualizer, save_videos, scene_flow_to_rgb
from datasets.utils import get_ground_np

logger = logging.getLogger()

import pickle, cv2
from tqdm import tqdm
import json

#---------------- Cityscapes semantic segmentation
cityscapes_classes = [
    "road", "sidewalk", "building", "wall", "fence", "pole",
    "traffic light", "traffic sign", "vegetation", "terrain", "sky",
    "person", "rider", "car", "truck", "bus", "train", "motorcycle",
    "bicycle"
]
cityscapes_classes_ind_map = {cn: i for i, cn in enumerate(cityscapes_classes)}

cityscapes_dynamic_classes = [
    "person", "rider", "car", "truck", "bus", "train", "motorcycle", "bicycle"
]

cityscapes_road_classes = [
    "road", "sidewalk", "building", "wall", "fence", "pole", "traffic light", "traffic sign"
]

cityscapes_human_classes = [
    "person", "rider"
]

total_camera_dict = {cam:i for i, cam in enumerate(["front_left", "front_right"])}

def load_pickle(path):
    with open(path, "rb") as f:
        return pickle.load(f, encoding="latin1")

def cal_shadow_mask(ret, ratio=[1.2, 1.0]):
    mask = ret.astype(np.uint8) * 255   # h,w
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        x_shift = x - (w * (ratio[0]-1)) / 2
        x_min, y_min = int(max(0, x_shift)), int(max(0, y))
        x_max, y_max = int(min(mask.shape[1], x_shift + ratio[1] * w)), int(min(mask.shape[0], y + ratio[0] * h))
        ret[y_min:y_max, x_min:x_max] = True
    return ret

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

class PlusPixelSource(ScenePixelSource):
    ORIGINAL_SIZE = [[540, 960], [540, 960]]

    def __init__(
        self,
        pixel_data_config: OmegaConf,
        data_path: str,
        start_timestep: int = 0,
        end_timestep: int = -1,
        device: torch.device = torch.device("cpu"),
        pose_type: str = "vio", 
        scene_id: str = None,
    ):
        super().__init__(pixel_data_config, device=device)
        self.data_path = data_path
        self.start_timestep = start_timestep
        self.end_timestep = end_timestep
        self.pose_type = pose_type
        self.scene_id = scene_id
        self.create_all_filelist()
        self.load_data()

    def create_all_filelist(self):
        """
        Create file lists for all data files.
        e.g., img files, feature files, etc.
        """
        if self.num_cams == 1:
            self.camera_list = ["front_left",]
        elif self.num_cams == 2:
            self.camera_list = ["front_left", "front_right"]
        else:
            raise NotImplementedError(
                f"num_cams: {self.num_cams} not supported for plus dataset"
            )
        
        data_file = os.path.join(self.data_path, "transforms.json")
        assert os.path.exists(data_file)
        with open(data_file) as f:
            data = json.load(f)        
        self.frames = data["frames"][self.start_timestep : self.end_timestep * self.num_cams]

        # ---- define filepaths ---- #
        img_filepaths, feat_filepaths = [], []
        mask_filepaths = []
        selected_steps = []

        # Note: we assume all the files in plus dataset are synchronized
        # for t in range(self.start_timestep, self.end_timestep):
        for f in self.frames:
            t = int(f["file_path"].split("/")[-1][:-4])
            selected_steps.append(t)

            cam_i = f["cam_name"]
            img_filepath = os.path.join(self.data_path, "images", cam_i, f"{t:06d}.png")
            if os.path.exists(img_filepath):
                img_filepaths.append(img_filepath)
            mask_filepath = os.path.join(self.data_path, "images", f"{cam_i}_mask", f"{t:06d}.npy")
            if not os.path.exists(mask_filepath):
                mask_filepath = os.path.join(self.data_path, "box_masks", cam_i, f"{t:06d}.png")
            if not os.path.exists(mask_filepath):
                mask_filepaths.append(None)
            else:
                mask_filepaths.append(mask_filepath)

        self.img_filepaths = np.array(img_filepaths)
        self.mask_filepaths = np.array(mask_filepaths)
        self.selected_steps = np.array(selected_steps)
    
    def load_calibrations(self):
        """
        Load the camera intrinsics, extrinsics, timestamps, etc.
        Compute the camera-to-world matrices, ego-to-world matrices, etc.
        """
        # to store per-camera intrinsics and extrinsics

        # compute per-image poses and intrinsics
        cam_ids = np.array([total_camera_dict[frame["cam_name"]] for frame in self.frames])
        intrs = np.array([frame["intr"] for frame in self.frames])

        if self.pose_type == 'odom':
            c2ws = np.array([frame["transform_matrix"] for frame in self.frames]) 
            self.cam_to_worlds = torch.from_numpy(c2ws)    # 避免eqdc的精度损失
        elif self.pose_type == 'vio':
            # temp for demo 
            # opencv2opengl = np.array([[1,0,0,0], [0,-1,0,0], [0,0,-1,0],[0,0,0,1]]).astype(float)
            # c2ws = np.array([frame["transform_matrix_vio"]@ opencv2opengl for frame in self.frames])

            c2ws = np.array([cam_pose_to_nerf(frame["transform_matrix_vio"], gl=False) for frame in self.frames])
            self.cam_to_worlds = torch.from_numpy(c2ws).float()
        else:
            raise ValueError("The pose_type is a ValueError str.")

        self.intrinsics = torch.from_numpy(intrs).float()
        self.ego_to_worlds = torch.FloatTensor([c2ws[i] @ np.linalg.inv(f["c2imu"]) for i, f in enumerate(self.frames)])
        self.cam_ids = torch.from_numpy(cam_ids).long()

        # the underscore here is important.
        self._timestamps = torch.from_numpy(self.selected_steps-self.start_timestep).float()
        self._timesteps = torch.from_numpy(self.selected_steps-self.start_timestep).long()

    def load_dynamic_mask(self) -> None:
        """
        Load the dynamic masks if they are available.
        """
        if not self.data_cfg.load_dynamic_mask:
            return
        dynamic_masks = []
        for fname in tqdm(
            self.mask_filepaths,
            desc="Loading dynamic masks",
            dynamic_ncols=True,
        ):
            if fname is not None:
                if fname.endswith(".npy"):
                    raw = np.load(fname)
                    ret = np.zeros_like(raw).astype(np.bool8)
                    for cls in cityscapes_dynamic_classes:
                        ind = cityscapes_classes_ind_map[cls]
                        ret[raw==ind] = True   
                    ret = ret.squeeze()                 
                elif fname.endswith(".png"):
                    ret = np.array(Image.open(fname).convert("L"))
                else:
                    raise TypeError("Error type of mask_file")
                # gen shadow mask for test, please set preload=true in config.yaml , ortherwise may reduce speed
                # ret = cal_shadow_mask(ret.squeeze())
                # resize them to the load_size
                dyn_mask = cv2.resize(ret.astype(int), (self.data_cfg.load_size[1], self.data_cfg.load_size[0]))
                dynamic_masks.append(np.array(dyn_mask) > 0)
            else:
                dynamic_masks.append(np.zeros((self.data_cfg.load_size[0], self.data_cfg.load_size[1])))
        self.dynamic_masks = torch.from_numpy(np.stack(dynamic_masks, axis=0)).float()


    def load_sky_mask(self) -> None:
        """
        Load the sky masks if they are available.
        """
        if not self.data_cfg.load_sky_mask:
            return
        sky_masks = []
        for fname in tqdm(
            self.mask_filepaths, desc="Loading sky masks", dynamic_ncols=True
        ):
            if fname is not None:
                if fname.endswith(".npy"):
                    raw = np.load(fname)
                    ret = np.zeros_like(raw).astype(np.bool8)
                    ret[raw==cityscapes_classes_ind_map["sky"]] = True
                    # resize them to the load_size
                    sky_mask = cv2.resize(ret.squeeze().astype(int), (self.data_cfg.load_size[1], self.data_cfg.load_size[0]))
                    sky_masks.append(np.array(sky_mask) > 0)
            else:
                sky_masks.append(np.zeros((self.data_cfg.load_size[1], self.data_cfg.load_size[0])))
        self.sky_masks = torch.from_numpy(np.stack(sky_masks, axis=0)).float()


class PlusLiDARSource(SceneLidarSource):
    def __init__(
        self,
        lidar_data_config: OmegaConf,
        data_path: str,
        start_timestep: int,
        end_timestep: int,
        device: torch.device = torch.device("cpu"),
        pose_type: str = "vio",
        scene_id: str = None,
    ):
        super().__init__(lidar_data_config, device=device)
        self.data_path = data_path
        self.pose_type = pose_type
        self.start_timestep = start_timestep
        self.end_timestep = end_timestep
        self.scene_id = scene_id
        self.create_all_filelist()
        self.load_data()

    def create_all_filelist(self):
        """
        Create a list of all the files in the dataset.
        e.g., a list of all the lidar scans in the dataset.
        """
        data_file = os.path.join(self.data_path, "transforms.json")
        assert os.path.exists(data_file)
        with open(data_file) as f:
            data = json.load(f)        
        self.frames = data["frames"][self.start_timestep : self.end_timestep * self.data_cfg.num_cams: self.data_cfg.num_cams]
        
        lidar_filepaths = []
        selected_steps = []
        for f in self.frames:
            t = int(f["file_path"].split("/")[-1][:-4])
            selected_steps.append(t)
            
            lidar_filepaths.append(
                os.path.join(self.data_path, "cloud", f"{t:06d}.bin")
            )
        self.lidar_filepaths = np.array(lidar_filepaths)
        self.selected_steps = np.array(selected_steps)

    def load_calibrations(self):
        """
        Load the calibration files of the dataset.
        e.g., lidar to world transformation matrices.
        """
        
        # we tranform the poses w.r.t. the first timestep to make the origin of the
        # first ego pose as the origin of the world coordinate system.
        l2imu = np.array(self.frames[0]["l2imu"])
        if "20230319T090808_pdb-l4e-b0007_6_871to931" in self.data_path:
            l2imu = np.array([
                [9.8429859096846628e-01, -1.5662345907265007e-02, 1.7581517209323497e-01, 5.1420710941564280e+00,],
                [1.2696898646099625e-02, 9.9975773963492853e-01, 1.7979176978685573e-02, -5.0000000000000003e-02,],
                [-1.7605417513442656e-01, -1.5464571146378534e-02, 9.8425899765102265e-01, 2.6510007479274904e+00],
                [0., 0., 0., 1.]
                ])
        if self.pose_type == 'odom':
            l2ws = np.array([frame["imu2w"] @ l2imu for frame in self.frames]) 
            self.lidar_to_worlds = torch.from_numpy(l2ws).float()    # 避免eqdc的精度损失
        elif self.pose_type == 'vio':
            # due to camera transform
            l2ws = np.array([cam_pose_to_nerf(frame["imu2w_vio"] @ l2imu, gl=False) for frame in self.frames])
            self.lidar_to_worlds = torch.from_numpy(l2ws).float()
        else:
            raise ValueError("The pose_type is a ValueError str.")


    def load_lidar(self):
        """
        Load the lidar data of the dataset from the filelist.
        """
        origins, directions, ranges, grounds, laser_ids = [], [], [], [], []
        # flow/ground info are used for evaluation only
        flows, flow_classes, grounds = [], [], []
        # in plus, we simplify timestamps as the time indices
        timestamps, timesteps = [], []
        accumulated_num_rays = 0
        for t in trange(
            0, len(self.lidar_filepaths), desc="Loading lidar", dynamic_ncols=True
        ):
            # each lidar_info contains an Nx14 array
            # from left to right:
            # origins: 3d, points: 3d, flows: 3d, flow_class: 1d,
            # ground_labels: 1d, intensities: 1d, elongations: 1d, laser_ids: 1d
            lidar_origins = torch.zeros(1,3)
            lidar_points = torch.from_numpy(np.fromfile(self.lidar_filepaths[t], dtype=np.float32).reshape(-1,4)[:,:3]).float()
            
            # transform lidar points from lidar coordinate system to world coordinate system
            lidar_origins = (
                self.lidar_to_worlds[t][:3, :3] @ lidar_origins.T
                + self.lidar_to_worlds[t][:3, 3:4]
            ).T 
            lidar_points = (
                self.lidar_to_worlds[t][:3, :3] @ lidar_points.T
                + self.lidar_to_worlds[t][:3, 3:4]
            ).T
            lidar_directions = lidar_points - lidar_origins
            lidar_ranges = torch.norm(lidar_directions, dim=-1, keepdim=True)
            lidar_directions = lidar_directions / lidar_ranges
            accumulated_num_rays += len(lidar_ranges)
            origins.append(lidar_origins.repeat(len(lidar_ranges), 1))
            directions.append(lidar_directions)
            ranges.append(lidar_ranges)
            # grounds.append(get_ground_np(lidar_points))

            # we use time indices as the timestamp for waymo dataset
            lidar_timestamp = torch.ones_like(lidar_ranges).squeeze(-1) * self.selected_steps[t]
            timestamps.append(lidar_timestamp)
            timesteps.append(lidar_timestamp)
        
        logger.info(
            f"Number of lidar rays: {accumulated_num_rays} "
        )
        self.origins = torch.cat(origins, dim=0)
        self.directions = torch.cat(directions, dim=0)
        self.ranges = torch.cat(ranges, dim=0)
        # self.grounds = torch.cat(grounds, dim=0)

        self._timestamps = (torch.cat(timestamps, dim=0)-self.start_timestep).float()
        self._timesteps = (torch.cat(timesteps, dim=0)-self.start_timestep).long()


class PlusDataset(SceneDataset):
    dataset: str = "plus"

    def __init__(
        self,
        data_cfg: OmegaConf,
    ) -> None:
        super().__init__(data_cfg)
        self.data_path = os.path.join(self.data_cfg.data_root, self.scene_idx)
        assert self.data_cfg.dataset == "plus"
        assert os.path.exists(self.data_path), f"{self.data_path} does not exist"
        print(f"Processing scene: {self.scene_idx}")

        # ---- find the number of synchronized frames ---- #
        if self.data_cfg.end_timestep == -1:
            files = sorted(os.listdir(os.path.join(self.data_path, "images", "front_left")))
            end_timestep = int(os.path.basename(files[-1])[:-4])    # 有时候vio-pose会丢帧
        else:
            end_timestep = self.data_cfg.end_timestep
        # to make sure the last timestep is included
        self.end_timestep = end_timestep + 1
        self.start_timestep = self.data_cfg.start_timestep

        # ---- create data source ---- #
        self.pixel_source, self.lidar_source = self.build_data_source()
        self.aabb = self.get_aabb()

        # ---- define train and test indices ---- #
        # note that the timestamps of the pixel source and the lidar source are the same in plus dataset
        (
            self.train_timesteps,
            self.test_timesteps,
            self.train_indices,
            self.test_indices,
        ) = self.split_train_test()

        # ---- create split wrappers ---- #
        pixel_sets, lidar_sets = self.build_split_wrapper()
        self.train_pixel_set, self.test_pixel_set, self.full_pixel_set = pixel_sets
        self.train_lidar_set, self.test_lidar_set, self.full_lidar_set = lidar_sets

    def build_split_wrapper(self):
        """
        Makes each data source as a Pytorch Dataset
        """
        train_pixel_set, test_pixel_set, full_pixel_set = None, None, None
        train_lidar_set, test_lidar_set, full_lidar_set = None, None, None

        if self.pixel_source is not None:
            train_pixel_set = SplitWrapper(
                datasource=self.pixel_source,
                # train_indices are img indices, so the length is num_cams * num_train_timesteps
                split_indices=self.train_indices,
                split="train",
                ray_batch_size=self.data_cfg.ray_batch_size,
            )
            full_pixel_set = SplitWrapper(
                datasource=self.pixel_source,
                # cover all the images
                split_indices=np.arange(self.pixel_source.num_imgs).tolist(),
                split="full",
                ray_batch_size=self.data_cfg.ray_batch_size,
            )
            if len(self.test_indices) > 0:
                test_pixel_set = SplitWrapper(
                    datasource=self.pixel_source,
                    # test_indices are img indices, so the length is num_cams * num_test_timesteps
                    split_indices=self.test_indices,
                    split="test",
                    ray_batch_size=self.data_cfg.ray_batch_size,
                )
        if self.lidar_source is not None:
            train_lidar_set = SplitWrapper(
                datasource=self.lidar_source,
                # train_timesteps are lidar indices, so the length is num_train_timesteps
                split_indices=self.train_timesteps,
                split="train",
                ray_batch_size=self.data_cfg.ray_batch_size,
            )
            full_lidar_set = SplitWrapper(
                datasource=self.lidar_source,
                # cover all the lidar scans
                split_indices=np.arange(self.lidar_source.num_timesteps).tolist(),
                split="full",
                ray_batch_size=self.data_cfg.ray_batch_size,
            )
            if len(self.test_indices) > 0:
                test_lidar_set = SplitWrapper(
                    datasource=self.lidar_source,
                    # test_timesteps are lidar indices, so the length is num_test_timesteps
                    split_indices=self.test_timesteps,
                    split="test",
                    ray_batch_size=self.data_cfg.ray_batch_size,
                )
        pixel_set = (train_pixel_set, test_pixel_set, full_pixel_set)
        lidar_set = (train_lidar_set, test_lidar_set, full_lidar_set)
        return pixel_set, lidar_set

    def build_data_source(self):
        """
        Create the data source for the dataset.
        """
        pixel_source, lidar_source = None, None
        # to collect all timestamps from pixel source and lidar source
        all_timestamps = []
        # ---- create pixel source ---- #
        load_pixel = (
            self.data_cfg.pixel_source.load_rgb
            or self.data_cfg.pixel_source.load_sky_mask
            or self.data_cfg.pixel_source.load_dynamic_mask
            or self.data_cfg.pixel_source.load_feature
        )

        if load_pixel:
            pixel_source = PlusPixelSource(
                self.data_cfg.pixel_source,
                self.data_path,
                self.start_timestep,
                self.end_timestep,
                device=self.device,
                pose_type=self.data_cfg.pose_type,
                scene_id=self.scene_idx,
            )
            pixel_source.to(self.device)
            # collect img timestamps
            all_timestamps.append(pixel_source.timestamps)
        
        if self.data_cfg.lidar_source.load_lidar:
            lidar_source = PlusLiDARSource(
                self.data_cfg.lidar_source,
                self.data_path,
                self.start_timestep,
                self.end_timestep,
                device=self.device,
                pose_type=self.data_cfg.pose_type,
                scene_id=self.scene_idx,
            )
            lidar_source.to(self.device)
            # collect lidar timestamps
            all_timestamps.append(lidar_source.timestamps)

        assert len(all_timestamps) > 0, "No data source is loaded"
        all_timestamps = torch.cat(all_timestamps, dim=0)
        # normalize the timestamps jointly for pixel source and lidar source
        # so that the normalized timestamps are between 0 and 1
        all_timestamps = (all_timestamps - all_timestamps.min()) / (
            all_timestamps.max() - all_timestamps.min()
        )
        if pixel_source is not None:
            pixel_source.register_normalized_timestamps(
                all_timestamps[: len(pixel_source.timestamps)]
            )
        if lidar_source is not None:
            lidar_source.register_normalized_timestamps(
                all_timestamps[-len(lidar_source.timestamps) :]
            )
        return pixel_source, lidar_source

    def split_train_test(self):
        if self.data_cfg.pixel_source.test_image_stride != 0:
            test_timesteps = np.arange(
                # it makes no sense to have test timesteps before the start timestep
                self.data_cfg.pixel_source.test_image_stride,
                self.num_img_timesteps,
                self.data_cfg.pixel_source.test_image_stride,
            )
        else:
            test_timesteps = []
        train_timesteps = np.array(
            [i for i in range(self.num_img_timesteps) if i not in test_timesteps]
        )
        logger.info(
            f"Train timesteps: \n{self.pixel_source.selected_steps[train_timesteps]}"
        )
        logger.info(
            f"Test timesteps: \n{self.pixel_source.selected_steps[test_timesteps]}"
        )
        # propagate the train and test timesteps to the train and test indices
        train_indices, test_indices = [], []
        for t in range(self.num_img_timesteps):
            if t in train_timesteps:
                for cam in range(self.pixel_source.num_cams):
                    train_indices.append(t * self.pixel_source.num_cams + cam)
            elif t in test_timesteps:
                for cam in range(self.pixel_source.num_cams):
                    test_indices.append(t * self.pixel_source.num_cams + cam)
        logger.info(f"Number of train indices: {len(train_indices)}")
        logger.info(f"Train indices: {train_indices}")
        logger.info(f"Number of test indices: {len(test_indices)}")
        logger.info(f"Test indices: {test_indices}")

        # Again, training and testing indices are indices into the full dataset
        # train_indices are img indices, so the length is num_cams * num_train_timesteps
        # but train_timesteps are timesteps, so the length is num_train_timesteps (len(unique_train_timestamps))
        return train_timesteps, test_timesteps, train_indices, test_indices

    def save_videos(self, video_dict: dict, **kwargs):
        """
        Save the a video of the data.
        """
        return save_videos(
            render_results=video_dict,
            save_pth=kwargs["save_pth"],
            num_timestamps=kwargs["num_timestamps"],
            keys=kwargs["keys"],
            num_cams=kwargs["num_cams"],
            fps=kwargs["fps"],
            verbose=kwargs["verbose"],
            save_seperate_video=kwargs["save_seperate_video"],
        )

    def render_data_videos(
        self,
        save_pth: str,
        split: str = "full",
        fps: int = 24,
        verbose=True,
    ):
        """
        Render a video of data.
        """
        pixel_dataset, lidar_dataset = None, None
        if split == "full":
            if self.pixel_source is not None:
                pixel_dataset = self.full_pixel_set
            if self.lidar_source is not None:
                lidar_dataset = self.full_lidar_set
        elif split == "train":
            if self.pixel_source is not None:
                pixel_dataset = self.train_pixel_set
            if self.lidar_source is not None:
                lidar_dataset = self.train_lidar_set
        elif split == "test":
            if self.pixel_source is not None:
                pixel_dataset = self.test_pixel_set
            if self.lidar_source is not None:
                lidar_dataset = self.test_lidar_set
        else:
            raise NotImplementedError(f"Split {split} not supported")

        # pixel source
        rgb_imgs, dynamic_objects = [], []
        sky_masks, feature_pca_colors = [], []
        lidar_depths, flow_colors = [], []

        for i in trange(
            len(pixel_dataset), desc="Rendering data videos", dynamic_ncols=True
        ):
            data_dict = pixel_dataset[i]
            if "pixels" in data_dict:
                rgb_imgs.append(data_dict["pixels"].cpu().numpy())
            if "dynamic_masks" in data_dict:
                dynamic_objects.append(
                    (data_dict["dynamic_masks"].unsqueeze(-1) * data_dict["pixels"])
                    .cpu()
                    .numpy()
                )
            if "sky_masks" in data_dict:
                sky_masks.append(data_dict["sky_masks"].cpu().numpy())
            if "features" in data_dict:
                features = data_dict["features"]
                # use registered parameters to normalize the features for visualization
                features = features @ self.pixel_source.feat_dimension_reduction_mat
                features = (features - self.pixel_source.feat_color_min) / (
                    self.pixel_source.feat_color_max - self.pixel_source.feat_color_min
                ).clamp(0, 1)
                feature_pca_colors.append(features.cpu().numpy())
            if lidar_dataset is not None:
                # to deal with asynchronized data
                # find the closest lidar scan to the current image in time
                closest_lidar_idx = self.lidar_source.find_closest_timestep(
                    data_dict["normed_timestamps"].flatten()[0]
                )
                data_dict = lidar_dataset[closest_lidar_idx]
                lidar_points = (
                    data_dict["lidar_origins"]
                    + data_dict["lidar_ranges"] * data_dict["lidar_viewdirs"]
                )
                # project lidar points to the image plane
                # TODO: consider making this a function
                intrinsic_4x4 = torch.nn.functional.pad(
                    self.pixel_source.intrinsics[i], (0, 0, 0, 1)
                )
                intrinsic_4x4[3, 3] = 1.0
                lidar2img = intrinsic_4x4 @ self.pixel_source.cam_to_worlds[i].inverse().float()  # lidar to img
                lidar_points = (
                    lidar2img[:3, :3] @ lidar_points.T + lidar2img[:3, 3:4]
                ).T
                depth = lidar_points[:, 2]
                cam_points = lidar_points[:, :2] / (depth.unsqueeze(-1) + 1e-6)
                valid_mask = (
                    (cam_points[:, 0] >= 0)
                    & (cam_points[:, 0] < self.pixel_source.WIDTH)
                    & (cam_points[:, 1] >= 0)
                    & (cam_points[:, 1] < self.pixel_source.HEIGHT)
                    & (depth > 0)
                )
                _depth = depth[valid_mask]
                _cam_points = cam_points[valid_mask]
                depth_map = torch.zeros(
                    self.pixel_source.HEIGHT, self.pixel_source.WIDTH
                ).to(self.device)
                depth_map[
                    _cam_points[:, 1].long(), _cam_points[:, 0].long()
                ] = _depth.squeeze(-1)
                depth_img = depth_map.cpu().numpy()
                depth_img = depth_visualizer(depth_img, depth_img > 0)
                mask = (depth_map.unsqueeze(-1) > 0).cpu().numpy()
                # show the depth map on top of the rgb image
                image = rgb_imgs[-1] * (1 - mask) + depth_img * mask
                lidar_depths.append(image)

                # show depth cmap
                import matplotlib.pyplot as plt
                import matplotlib.image as mpimg                
                # pts = np.fromfile(self.lidar_source.lidar_filepaths[i], dtype=np.float32).reshape(-1,4)
                # pts[:, 3] = 1.
                # P = np.array(self.pixel_source.intrinsics[i])
                # c2w = np.array(self.pixel_source.cam_to_worlds[i])
                # l2w = np.array(self.lidar_source.lidar_to_worlds[i])
                # lidar2img = P @ np.linalg.inv(c2w) @ l2w
                # proj = lidar2img @ pts.T
                # cam = np.delete(proj,np.where(proj[2,:]<=0),axis=1)
                # cam[:2,:] /= cam[2,:]
                # u,v,z = cam
                # u_out = np.logical_or(u<0, u>960)
                # v_out = np.logical_or(v<0, v>540)
                # outlier = np.logical_or(u_out, v_out)
                # cam = np.delete(cam,np.where(outlier),axis=1)
                # u,v,z = cam
                u,v,z = _cam_points[:, 0].long().numpy(), _cam_points[:, 1].long().numpy(), _depth.numpy()
                plt.imshow(rgb_imgs[-1])
                plt.scatter([u], [v], c=[z],cmap='rainbow',alpha=0.5,s=2)
                plt.savefig("tmp.png", bbox_inches='tight')

                # project lidar flows to the image plane
                # # to examine whether the ground labels are correct
                # # valid_mask = valid_mask & (~data_dict["lidar_ground"])
                # _cam_points = cam_points[valid_mask]
                # # final color:
                # #  white if no flow, black if ground, and flow color otherwise
                # flow_color = scene_flow_to_rgb(
                #     data_dict["lidar_flow"][valid_mask],
                #     background="bright",
                #     flow_max_radius=1.0,
                # )
                # flow_img[
                #     _cam_points[:, 1].long(), _cam_points[:, 0].long()
                # ] = flow_color
                # flow_img = flow_img.cpu().numpy()
                # mask = (depth_map.unsqueeze(-1) > 0).cpu().numpy()
                # # # show the flow on top of the rgb image
                # image = rgb_imgs[-1] * (1 - mask) + flow_img.cpu().numpy() * mask
                # flow_colors.append(image)

        video_dict = {
            # "gt_rgbs": rgb_imgs,
            "stacked": lidar_depths,
            # "flow_colors": flow_colors,
            # "gt_feature_pca_colors": feature_pca_colors,
            # "gt_dynamic_objects": dynamic_objects,
            # "gt_sky_masks": sky_masks,
        }
        video_dict = {k: v for k, v in video_dict.items() if len(v) > 0}
        return self.save_videos(
            video_dict,
            save_pth=save_pth,
            num_timestamps=self.num_img_timesteps,
            keys=video_dict.keys(),
            num_cams=self.pixel_source.num_cams,
            fps=fps,
            verbose=verbose,
            save_seperate_video=False,
        )
    
# 计算平均SE(3)变换
def average_se3_transformations(transformations):
    from scipy.spatial.transform import Rotation as R   
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
        
class ScenarioDataset(SceneDataset):
    dataset: str = "plus"

    def __init__(
        self,
        data_cfg: OmegaConf,
    ) -> None:
        super().__init__(data_cfg)
        assert self.data_cfg.dataset == "plus"

        self.scenes = {}
        c2ws = []
        if "scenarios" in data_cfg:
            for scenario_cfg_str in data_cfg.scenarios:
                _scene_id, start, stop = [it.strip(" ") for it in scenario_cfg_str.split(",")]
                cfg = data_cfg.copy()
                cfg.scene_idx = _scene_id
                cfg.start_timestep, cfg.end_timestep = int(start), int(stop)
                dataset = PlusDataset(data_cfg=cfg)
                self.scenes[_scene_id] = dataset
                c2ws.append(dataset.pixel_source.cam_to_worlds.numpy())
        c2ws = np.concatenate(c2ws)

        self.base_pose = np.eye(4)
        if cfg.pose_type == 'odom':
            # rebase the pose and aabb_box of scenario dataset, for multi bag
            self.base_pose = average_se3_transformations(c2ws)  # ? base eqdc pose
            aabb_min, aabb_max = -1*torch.ones(3), torch.ones(3)
            for scene_id, dataset in self.scenes.items():
                # rebase pose, and transform from opencv coord to world coord
                c2ws_rebase = [cam_pose_to_nerf(np.linalg.inv(self.base_pose) @ c2w, gl=False) for c2w in dataset.pixel_source.cam_to_worlds.numpy()]
                dataset.pixel_source.cam_to_worlds = torch.FloatTensor(c2ws_rebase)
                dataset.aabb = dataset.get_aabb()
                # cal scenario_aabb
                aabb_min[aabb_min > dataset.aabb[:3]] =  dataset.aabb[:3][aabb_min > dataset.aabb[:3]]
                aabb_max[aabb_max < dataset.aabb[3:]] =  dataset.aabb[3:][aabb_max < dataset.aabb[3:]]
            self.aabb = torch.concatenate([aabb_min, aabb_max])
        elif cfg.pose_type == 'vio':
            # only support single bag
            assert len(self.scenes) == 1
            self.base_pose = c2ws[0]
        else:
            raise ValueError("The pose_type is a ValueError str.")

        print(f"Using the pose_type: \n {cfg.pose_type}")
        print(f"Scenario base_pose: \n {self.base_pose}") 
        # ---- create split wrappers ---- #
        self.build_split_wrapper()
        self.pixel_source = {scene_id: dataset.pixel_source for scene_id, dataset in self.scenes.items()}
        self.lidar_source = {scene_id: dataset.pixel_source for scene_id, dataset in self.scenes.items()}

    @property
    def num_cams(self) -> int:
        return self.data_cfg.pixel_source.num_cams

    @property
    def num_train_timesteps(self) -> Dict[str, int]:
        return {scene_id: len(dataset.train_timesteps) for scene_id, dataset in self.scenes.items()}

    @property
    def unique_normalized_training_timestamps(self) -> Dict[str, Tensor]:
        return {
            scene_id: dataset.pixel_source.unique_normalized_timestamps[dataset.train_timesteps]
            for scene_id, dataset in self.scenes.items()
            }
    
    @property
    def num_img_timesteps(self) -> Dict[str, int]:
        return {scene_id: dataset.pixel_source.num_timesteps for scene_id, dataset in self.scenes.items()}
    
    def build_split_wrapper(self):
        """
        Concat SplitWrapper(Pytorch Dataset) from different scenes
        """
        names = ["train_pixel_set", "test_pixel_set", "full_pixel_set", "train_lidar_set", "test_lidar_set", "full_lidar_set"]

        for name in names:
            setattr(self, name, [])
            for scene_id, dataset in self.scenes.items():
                if hasattr(dataset, name) and getattr(dataset, name) is not None:
                    data = getattr(dataset, name)
                    getattr(self, name).append(data)
            concat_dataset = getattr(self, name)
            concat_dataset = ConcatSplitWrapper(concat_dataset) if len(concat_dataset) > 0 else None
            setattr(self, name, concat_dataset)
                
    def build_data_source(self):
        pass

    def split_train_test(self):
        pass

    def save_videos(self, video_dict: dict, **kwargs):
        """
        Save the a video of the data.
        """
        pass

    def render_data_videos(
        self,
        save_pth: str,
        split: str = "full",
        fps: int = 24,
        verbose=True,
    ):
        """
        Render a video of data.
        """

        # for scene_id, dataset in scenarios:
        #     save_pth = os.path.join(cfg.log_dir, f"{scene_id}_data.mp4")
        pass


class ConcatSplitWrapper(ConcatDataset):
    def __init__(self, datasets: List[SplitWrapper]):
        super(ConcatSplitWrapper, self).__init__(datasets)

    def get_dataset_idx(self, idx):
        """
        获取索引为 idx 的样本所属的原始数据集索引
        """
        cumulative_sizes = [0] + self.cumulative_sizes
        for i, size in enumerate(cumulative_sizes[:-1]):
            if idx >= size and idx < cumulative_sizes[i + 1]:
                return i
        raise ValueError("Index out of range")
    
    def get_scene_id(self, idx):
        dataset_idx = self.get_dataset_idx(idx)
        datasource = self.datasets[dataset_idx].datasource
        if hasattr(datasource, "scene_id"):
            return getattr(datasource, "scene_id")
        else:
            return ""