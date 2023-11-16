import logging
import os
from typing import Dict

import numpy as np
import torch
from omegaconf import OmegaConf
from torch import Tensor
from tqdm import trange

from datasets.base.lidar_source import SceneLidarSource
from datasets.base.pixel_source import ScenePixelSource
from datasets.base.scene_dataset import SceneDataset
from datasets.base.split_wrapper import SplitWrapper
from datasets.utils import voxel_coords_to_world_coords
from radiance_fields.video_utils import depth_visualizer, save_videos, scene_flow_to_rgb

logger = logging.getLogger()

import pickle, cv2
from tqdm import tqdm

#---------------- Cityscapes semantic segmentation
cityscapes_classes = [
    'road', 'sidewalk', 'building', 'wall', 'fence', 'pole',
    'traffic light', 'traffic sign', 'vegetation', 'terrain', 'sky',
    'person', 'rider', 'car', 'truck', 'bus', 'train', 'motorcycle',
    'bicycle'
]
cityscapes_classes_ind_map = {cn: i for i, cn in enumerate(cityscapes_classes)}

cityscapes_dynamic_classes = [
    'person', 'rider', 'car', 'truck', 'bus', 'train', 'motorcycle', 'bicycle'
]

cityscapes_road_classes = [
    'road', 'sidewalk', 'building', 'wall', 'fence', 'pole', 'traffic light', 'traffic sign'
]

cityscapes_human_classes = [
    'person', 'rider'
]

total_camera_dict = {cam:i for i, cam in enumerate(['front_left', 'front_right'])}

def load_pickle(path):
    with open(path, 'rb') as f:
        return pickle.load(f, encoding='latin1')

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


class PlusPixelSource(ScenePixelSource):
    ORIGINAL_SIZE = [[540, 960], [540, 960]]

    def __init__(
        self,
        pixel_data_config: OmegaConf,
        data_path: str,
        start_timestep: int = 0,
        end_timestep: int = -1,
        device: torch.device = torch.device("cpu"),
        pose_type: str = 'vio',
    ):
        super().__init__(pixel_data_config, device=device)
        self.data_path = data_path
        self.start_timestep = start_timestep
        self.end_timestep = end_timestep
        self.pose_type = pose_type
        self.create_all_filelist()
        self.load_data()

    def create_all_filelist(self):
        """
        Create file lists for all data files.
        e.g., img files, feature files, etc.
        """
        if self.num_cams == 1:
            self.camera_list = ['front_left',]
        elif self.num_cams == 2:
            self.camera_list = ['front_left', 'front_right']
        else:
            raise NotImplementedError(
                f"num_cams: {self.num_cams} not supported for plus dataset"
            )

        # ---- define filepaths ---- #
        img_filepaths, feat_filepaths = [], []
        mask_filepaths = []
        selected_steps = []    # 可用帧的数据在self.start_timestep到self.end_timestep中的indice

        # Note: we assume all the files in plus dataset are synchronized
        for t in range(self.start_timestep, self.end_timestep):
            for cam_i in self.camera_list:
                img_filepath = os.path.join(self.data_path, "images", cam_i, f"{t:06d}.png")
                mask_filepath = os.path.join(self.data_path, "images", f'{cam_i}_mask', f"{t:06d}.npy")
                if os.path.exists(img_filepath):
                    img_filepaths.append(img_filepath)
                    selected_steps.append(t)
                if os.path.exists(mask_filepath):
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
        if self.pose_type == 'imu':
            # imu ego pose
            ego_poses_path = os.path.join(self.data_path, 'ego_pos_with_vel')
            poses_imu_w_tracking = []
            for frame in range(len(os.listdir(ego_poses_path))):
                info = load_pickle(os.path.join(ego_poses_path, ("%06d"%frame)+".pkl"))
                poses_imu_w_tracking.append(info['ego_pose']) 
            poses_imu_w_tracking = np.array(poses_imu_w_tracking).astype(np.float64)
            pose0 = poses_imu_w_tracking[0].copy()
            poses_imu_w_tracking = np.linalg.inv(pose0) @ poses_imu_w_tracking
            
            # calib
            calibration_file = os.path.join(self.data_path, "calib", ("%06d"%0)+".pkl")
            calib = load_pickle(calibration_file)
            """ 
            calib参数介绍
                M: 原始内参矩阵; D: 畸变参数; P: 去完畸变的新内参矩阵; R:image平面->rectify平面的旋转矩阵 ; Tr_cam_to_imu: camera->imu的外参矩阵;
            """
            cam_ids, intrs, c2ws = [], [], []
            for t in range(self.start_timestep, self.end_timestep):
                for cam_i in self.camera_list:
                    cam_ids.append(total_camera_dict[cam_i])
                    intrs.append(calib['P2'] if cam_i=='front_right' else calib['P1'])
                    
                    cam_i_imu = calib[f"Tr_cam_to_imu_{cam_i}"]
                    c2ws.append(poses_imu_w_tracking[t] @ cam_i_imu)
            cam_ids, intrs, c2ws = np.array(cam_ids), np.array(intrs), np.array(c2ws), 

        elif self.pose_type == 'vio':
            import json
            opencv2opengl = np.array([[1,0,0,0], [0,-1,0,0], [0,0,-1,0],[0,0,0,1]]).astype(float)
            opengl2opencv = np.linalg.inv(opencv2opengl)
            data_file = os.path.join(self.data_path, "transforms.json")
            assert os.path.exists(data_file)
            with open(data_file) as f:
                data = json.load(f)
            frames = data['frames']
            cam_ids = np.array([total_camera_dict[frame['cam_name']] for frame in frames])
            intrs = np.array([frame['intr'] for frame in frames])
            c2ws = np.array([frame['transform_matrix'] @ opengl2opencv for frame in frames]) # opengl to opencv

        indices = self.selected_steps * self.num_cams
        for i in range(self.num_cams):
            indices[i::self.num_cams] += i

        self.intrinsics = torch.from_numpy(intrs).float()[indices]
        self.cam_to_worlds = torch.from_numpy(c2ws).float()[indices]
        # self.ego_to_worlds = torch.from_numpy(poses_imu_w_tracking).float()
        self.cam_ids = torch.from_numpy(cam_ids).long()[indices]

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
            raw = np.load(fname)
            ret = np.zeros_like(raw).astype(np.bool8)
            for cls in cityscapes_dynamic_classes:
                ind = cityscapes_classes_ind_map[cls]
                ret[raw==ind] = True
            
            # gen shadow mask for test, please set preload=true in config.yaml , ortherwise may reduce speed
            ret = cal_shadow_mask(ret.squeeze())
            # resize them to the load_size
            dyn_mask = cv2.resize(ret.astype(int), (self.data_cfg.load_size[1], self.data_cfg.load_size[0]))
            dynamic_masks.append(np.array(dyn_mask) > 0)
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
            raw = np.load(fname)
            ret = np.zeros_like(raw).astype(np.bool8)
            ret[raw==cityscapes_classes_ind_map['sky']] = True
            # resize them to the load_size
            sky_mask = cv2.resize(ret.squeeze().astype(int), (self.data_cfg.load_size[1], self.data_cfg.load_size[0]))
            sky_masks.append(np.array(sky_mask) > 0)
        self.sky_masks = torch.from_numpy(np.stack(sky_masks, axis=0)).float()


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
            # pose_type = self.data_cfg.pose_type if 'pose_type' in self.data_cfg
            pixel_source = PlusPixelSource(
                self.data_cfg.pixel_source,
                self.data_path,
                self.start_timestep,
                self.end_timestep,
                device=self.device,
            )
            pixel_source.to(self.device)
            # collect img timestamps
            all_timestamps.append(pixel_source.timestamps)

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
                    self.pixel_source.intrinsics[i], (0, 1, 0, 1)
                )
                intrinsic_4x4[3, 3] = 1.0
                lidar2img = intrinsic_4x4 @ self.pixel_source.cam_to_worlds[i].inverse()
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
                depth = depth[valid_mask]
                _cam_points = cam_points[valid_mask]
                depth_map = torch.zeros(
                    self.pixel_source.HEIGHT, self.pixel_source.WIDTH
                ).to(self.device)
                depth_map[
                    _cam_points[:, 1].long(), _cam_points[:, 0].long()
                ] = depth.squeeze(-1)
                depth_img = depth_map.cpu().numpy()
                depth_img = depth_visualizer(depth_img, depth_img > 0)
                mask = (depth_map.unsqueeze(-1) > 0).cpu().numpy()
                # show the depth map on top of the rgb image
                image = rgb_imgs[-1] * (1 - mask) + depth_img * mask
                lidar_depths.append(image)

                # project lidar flows to the image plane
                flow_img = torch.zeros(
                    self.pixel_source.HEIGHT, self.pixel_source.WIDTH, 3
                ).to(self.device)
                # to examine whether the ground labels are correct
                valid_mask = valid_mask & (~data_dict["lidar_ground"])
                _cam_points = cam_points[valid_mask]
                # final color:
                #  white if no flow, black if ground, and flow color otherwise
                flow_color = scene_flow_to_rgb(
                    data_dict["lidar_flow"][valid_mask],
                    background="bright",
                    flow_max_radius=1.0,
                )
                flow_img[
                    _cam_points[:, 1].long(), _cam_points[:, 0].long()
                ] = flow_color
                flow_img = flow_img.cpu().numpy()
                mask = (depth_map.unsqueeze(-1) > 0).cpu().numpy()
                # show the flow on top of the rgb image
                image = rgb_imgs[-1] * (1 - mask) + flow_img * mask
                flow_colors.append(image)

        video_dict = {
            "gt_rgbs": rgb_imgs,
            "stacked": lidar_depths,
            "flow_colors": flow_colors,
            "gt_feature_pca_colors": feature_pca_colors,
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
