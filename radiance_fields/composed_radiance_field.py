import logging
from typing import Callable, Dict, List, Literal, Optional, Tuple, Union

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch import Tensor

from radiance_fields.encodings import (
    HashEncoder,
    SinusoidalEncoder,
    build_xyz_encoder_from_cfg,
)
from radiance_fields.mlp import MLP
from radiance_fields.nerf_utils import contract, trunc_exp
from radiance_fields.radiance_field import temporal_interpolation


logger = logging.getLogger()


class ComposedRadianceField(nn.Module):
    def __init__(
        self,
        xyz_encoder: HashEncoder,
        dynamic_xyz_encoders: Dict[str, Optional[HashEncoder]],
        flow_xyz_encoders: Dict[str, Optional[HashEncoder]],
        aabb: Union[Tensor, List[float]] = [-1, -1, -1, 1, 1, 1],
        num_dims: int = 3,
        density_activation: Callable = lambda x: trunc_exp(x - 1),
        unbounded: bool = True,
        geometry_feature_dim: int = 15,
        base_mlp_layer_width: int = 64,
        head_mlp_layer_width: int = 64,
        enable_cam_embedding: bool = False,
        enable_img_embedding: bool = False,
        num_cams: int = 3,
        appearance_embedding_dim: int = 16,
        semantic_feature_dim: int = 64,
        feature_mlp_layer_width: int = 256,
        feature_embedding_dim: int = 768,
        enable_sky_head: bool = False,
        enable_shadow_head: bool = False,
        enable_feature_head: bool = False,
        num_train_timesteps: Dict[str, int] = {},
        interpolate_xyz_encoding: bool = False,
        enable_learnable_pe: bool = True,
        enable_temporal_interpolation: bool = False
    ) -> None:
        super().__init__()
        # scene properties
        if not isinstance(aabb, Tensor):
            aabb = torch.tensor(aabb, dtype=torch.float32)
        self.register_buffer("aabb", aabb)
        self.unbounded = unbounded
        self.num_cams = num_cams
        self.num_dims = num_dims
        self.density_activation = density_activation

        # appearance embedding
        self.enable_cam_embedding = enable_cam_embedding
        self.enable_img_embedding = enable_img_embedding
        self.appearance_embedding_dim = appearance_embedding_dim

        self.geometry_feature_dim = geometry_feature_dim
        # add semantic feature dim if feature head is enabled
        if not enable_feature_head:
            semantic_feature_dim = 0
        self.semantic_feature_dim = semantic_feature_dim

        # note: we use very conservative default values for mlps
        # usually you want to use larger ones


        # ======== Static Field ======== #
        self.xyz_encoder = xyz_encoder
        self.base_mlp = nn.Sequential(
            nn.Linear(self.xyz_encoder.n_output_dims, base_mlp_layer_width),
            nn.ReLU(),
            nn.Linear(
                base_mlp_layer_width, geometry_feature_dim + semantic_feature_dim
            ),
        )

        # ======== Dynamic Field ======== #
        self.interpolate_xyz_encoding = interpolate_xyz_encoding
        self.dynamic_xyz_encoder = dynamic_xyz_encoders
        self.enable_temporal_interpolation = enable_temporal_interpolation

        self.dynamic_base_mlp = nn.ParameterDict()
        if len(self.dynamic_xyz_encoder) > 0:
            self.training_timesteps = {}
            for scene_id, dynamic_xyz_encoder in self.dynamic_xyz_encoder.items():
                # self.register_buffer("training_timesteps", torch.zeros(num_train_timesteps))
                # for temporal interpolation
                self.training_timesteps[scene_id] =  torch.zeros(num_train_timesteps[scene_id])
                self.dynamic_base_mlp[scene_id] = nn.Sequential(
                    nn.Linear(dynamic_xyz_encoder.n_output_dims, base_mlp_layer_width),
                    nn.ReLU(),
                    nn.Linear(
                        base_mlp_layer_width,
                        geometry_feature_dim + semantic_feature_dim,
                    ),
                )

        # ======== Flow Field ======== #
        self.flow_xyz_encoder = flow_xyz_encoders
        self.flow_mlp = nn.ParameterDict()
        if len(self.flow_xyz_encoder) > 0:
            for scene_id, flow_xyz_encoder in self.flow_xyz_encoder.items():
                self.flow_mlp[scene_id] = nn.Sequential(
                    nn.Linear(
                        flow_xyz_encoder.n_output_dims,
                        base_mlp_layer_width,
                    ),
                    nn.ReLU(),
                    nn.Linear(base_mlp_layer_width, base_mlp_layer_width),
                    nn.ReLU(),
                    nn.Linear(base_mlp_layer_width, 6),  # 3 for forward, 3 for backward
                    # no activation function for flow
                )

        # appearance embedding
        if self.enable_cam_embedding:
            # per-camera embedding
            self.appearance_embedding = nn.Embedding(num_cams, appearance_embedding_dim)
        elif self.enable_img_embedding:
            # per-image embedding
            self.appearance_embedding = nn.ParameterDict()
            for scene_id, num_train_timestep in num_train_timesteps.items():
                self.appearance_embedding[scene_id] = nn.Embedding(
                    num_train_timestep * num_cams, appearance_embedding_dim
                )
        else:
            self.appearance_embedding = None

        # direction encoding
        self.direction_encoding = SinusoidalEncoder(
            n_input_dims=3, min_deg=0, max_deg=4
        )

        # ======== Color Head ======== #
        self.rgb_head = MLP(
            in_dims=geometry_feature_dim
            + self.direction_encoding.n_output_dims
            + (
                appearance_embedding_dim
                if self.enable_cam_embedding or self.enable_img_embedding
                else 0
            ),
            out_dims=3,
            num_layers=3,
            hidden_dims=head_mlp_layer_width,
            skip_connections=[1],
        )

        # ======== Shadow Head ======== #
        self.enable_shadow_head = enable_shadow_head
        if self.enable_shadow_head:
            self.shadow_head = nn.Sequential(
                nn.Linear(geometry_feature_dim, base_mlp_layer_width),
                nn.ReLU(),
                nn.Linear(base_mlp_layer_width, 1),
                nn.Sigmoid(),
            )

        # ======== Sky Head ======== #
        self.enable_sky_head = enable_sky_head
        if self.enable_sky_head:
            self.sky_head = MLP(
                in_dims=self.direction_encoding.n_output_dims
                + (
                    appearance_embedding_dim
                    if self.enable_cam_embedding or self.enable_img_embedding
                    else 0
                ),
                out_dims=3,
                num_layers=3,
                hidden_dims=head_mlp_layer_width,
                skip_connections=[1],
            )
            if enable_feature_head:
                # feature sky head
                raise NotImplementedError
                self.dino_sky_head = nn.Sequential(
                    # TODO: remove appearance embedding from dino sky head
                    nn.Linear(
                        self.direction_encoding.n_output_dims
                        + (
                            appearance_embedding_dim
                            if self.enable_cam_embedding or self.enable_img_embedding
                            else 0
                        ),
                        feature_mlp_layer_width,
                    ),
                    nn.ReLU(),
                    nn.Linear(feature_mlp_layer_width, feature_mlp_layer_width),
                    nn.ReLU(),
                    nn.Linear(feature_mlp_layer_width, feature_embedding_dim),
                )

        # ======== Feature Head ======== #
        self.enable_feature_head = enable_feature_head
        if self.enable_feature_head:
            raise NotImplementedError
            self.dino_head = nn.Sequential(
                nn.Linear(semantic_feature_dim, feature_mlp_layer_width),
                nn.ReLU(),
                nn.Linear(feature_mlp_layer_width, feature_mlp_layer_width),
                nn.ReLU(),
                nn.Linear(feature_mlp_layer_width, feature_embedding_dim),
            )
            # placeholders for visualization, will be registered when available
            self.register_buffer(
                "feats_reduction_mat", torch.zeros(feature_embedding_dim, 3)
            )
            self.register_buffer("feat_color_min", torch.zeros(3, dtype=torch.float32))
            self.register_buffer("feat_color_max", torch.ones(3, dtype=torch.float32))

            # positional embedding (PE) decomposition
            self.enable_learnable_pe = enable_learnable_pe
            if self.enable_learnable_pe:
                # globally-shared low-resolution learnable PE map
                self.learnable_pe_map = nn.Parameter(
                    0.05 * torch.randn(1, feature_embedding_dim // 2, 80, 120),
                    requires_grad=True,
                )
                # a PE head to decode PE features
                self.pe_head = nn.Sequential(
                    nn.Linear(feature_embedding_dim // 2, feature_embedding_dim),
                )

    def register_normalized_training_timesteps(
        self, normalized_timesteps: Dict[str, Tensor], time_diff: Dict[str, float]
    ) -> None:
        """
        register normalized timesteps for temporal interpolation

        Args:
            normalized_timesteps (Tensor): normalized timesteps in [0, 1]
            time_diff (float, optional): time difference between two consecutive timesteps. Defaults to None.
        """
        if len(self.dynamic_xyz_encoder) > 0:
            # register timesteps for temporal interpolation
            for scene_id, nt in normalized_timesteps.items():
                self.training_timesteps[scene_id] = nt.clone().to(self.device)
                
            if time_diff is not None:
                # use the provided time difference if available
                self.time_diff = time_diff
            else:
                # otherwise, compute the time difference from the provided timesteps
                # it's important to make sure the provided timesteps are consecutive
                self.time_diff = {}
                for scene_id, nt in self.training_timesteps.items():
                    self.time_diff[scene_id] = nt[1]-nt[0] if len(nt) > 1 else 0

    def set_aabb(self, aabb: Union[Tensor, List[float]]) -> None:
        """
        register aabb for scene space
        """
        if not isinstance(aabb, Tensor):
            aabb = torch.tensor(aabb, dtype=torch.float32)
        logger.info(f"Set aabb from {self.aabb} to {aabb}")
        self.aabb.copy_(aabb)
        self.aabb = self.aabb.to(self.device)

    def register_feats_reduction_mat(
        self,
        feats_reduction_mat: Tensor,
        feat_color_min: Tensor,
        feat_color_max: Tensor,
    ) -> None:
        """
        A placeholder for registering the PCA reduction matrix and min/max values for visualization.
        You may not want to compute PCA reduction matrix every time from the dataset.
        """
        # for visualization
        self.feats_reduction_mat.copy_(feats_reduction_mat)
        self.feat_color_min.copy_(feat_color_min)
        self.feat_color_max.copy_(feat_color_max)
        self.feats_reduction_mat = self.feats_reduction_mat.to(self.device)
        self.feat_color_min = self.feat_color_min.to(self.device)
        self.feat_color_max = self.feat_color_max.to(self.device)

    @property
    def device(self) -> torch.device:
        return self.aabb.device

    def contract_points(
        self,
        positions: Tensor,
    ) -> Tensor:
        """
        contract [-inf, inf] points to the range [0, 1] for hash encoding

        Returns:
            normed_positions: [..., 3] in [0, 1]
        """
        if self.unbounded:
            # use infinte norm to contract the positions for cuboid aabb
            normed_positions = contract(positions, self.aabb, ord=float("inf"))
        else:
            aabb_min, aabb_max = torch.split(self.aabb, 3, dim=-1)
            normed_positions = (positions - aabb_min) / (aabb_max - aabb_min)
        selector = (
            ((normed_positions > 0.0) & (normed_positions < 1.0))
            .all(dim=-1)
            .to(positions)
        )
        normed_positions = normed_positions * selector.unsqueeze(-1)
        return normed_positions

    def forward_static_hash(
        self,
        positions: Tensor,
    ) -> Tensor:
        """
        forward pass for static hash encoding

        Returns:
            encoded_features: [..., geometry_feature_dim + (semantic_feature_dim)]
            normed_positions: [..., 3] in [0, 1]
        """
        normed_positions = self.contract_points(positions)
        xyz_encoding = self.xyz_encoder(normed_positions.view(-1, self.num_dims))
        encoded_features = self.base_mlp(xyz_encoding).view(
            list(normed_positions.shape[:-1]) + [-1]
        )
        return encoded_features, normed_positions

    def forward_dynamic_hash(
        self,
        scene_id: str,
        normed_positions: Tensor,
        normed_timestamps: Tensor,
        return_hash_encodings: bool = False,
    ) -> Union[Tuple[Tensor, Tensor], Tensor]:
        """
        forward pass for dynamic hash encoding

        Returns:
            encoded_dynamic_feats: [..., geometry_feature_dim + (semantic_feature_dim)]
            dynamic_xyz_encoding: [..., n_output_dims] (optional)
        """
        if normed_timestamps.shape[-1] != 1:
            normed_timestamps = normed_timestamps.unsqueeze(-1)
        # To be fixed.
        # if self.training or not self.enable_temporal_interpolation:
        if True:
            temporal_positions = torch.cat(
                [normed_positions, normed_timestamps], dim=-1
            )
            dynamic_xyz_encoding = self.dynamic_xyz_encoder[scene_id](
                temporal_positions.view(-1, self.num_dims + 1)
            ).view(list(temporal_positions.shape[:-1]) + [-1])
            encoded_dynamic_feats = self.dynamic_base_mlp[scene_id](dynamic_xyz_encoding)
        else:
            encoded_dynamic_feats = temporal_interpolation(
                normed_timestamps,
                self.training_timesteps,
                normed_positions,
                self.dynamic_xyz_encoder,
                self.dynamic_base_mlp,
                interpolate_xyz_encoding=self.interpolate_xyz_encoding,
            )
        if return_hash_encodings:
            return encoded_dynamic_feats, dynamic_xyz_encoding
        else:
            return encoded_dynamic_feats

    def forward_flow_hash(
        self,
        scene_id: str,
        normed_positions: Tensor,
        normed_timestamps: Tensor,
    ) -> Tuple[Tensor, Tensor]:
        """
        forward pass for flow hash encoding

        Returns:
            flow: [..., 6] (forward_flow, backward_flow)
        """
        if normed_timestamps.shape[-1] != 1:
            normed_timestamps = normed_timestamps.unsqueeze(-1)
        if self.training or not self.enable_temporal_interpolation:
            temporal_positions = torch.cat(
                [normed_positions, normed_timestamps], dim=-1
            )
            flow_xyz_encoding = self.flow_xyz_encoder[scene_id](
                temporal_positions.view(-1, self.num_dims + 1)
            ).view(list(temporal_positions.shape[:-1]) + [-1])
            flow = self.flow_mlp[scene_id](flow_xyz_encoding)
        else:
            flow = temporal_interpolation(
                normed_timestamps,
                self.training_timesteps,
                normed_positions,
                self.flow_xyz_encoder,
                self.flow_mlp,
                interpolate_xyz_encoding=True,
            )
        return flow

    def forward(
        self,
        positions: Tensor,
        directions: Tensor = None,
        data_dict: Dict[str, Tensor] = {},
        return_density_only: bool = False,
        combine_static_dynamic: bool = False,
        query_feature_head: bool = True,
        query_pe_head: bool = True,
    ) -> Dict[str, Tensor]:
        """
        Args:
            positions: [..., 3]
            directions: [..., 3]
            data_dict: a dictionary containing additional data
            return_density_only: if True, only return density without querying other heads
            combine_static_dynamic: if True, combine static and dynamic predictions based on static and dynamic density
            in addition to returning separate results for static and dynamic fields
            query_feature_head: if True, query feature head
            query_pe_head: if True, query PE head. Disable this if we want to directly query 3D features.
        Returns:
            results_dict: a dictionary containing everything
        """
        results_dict = {}
        # forward static branch
        encoded_features, normed_positions = self.forward_static_hash(positions)
        geo_feats, semantic_feats = torch.split(
            encoded_features,
            [self.geometry_feature_dim, self.semantic_feature_dim],
            dim=-1,
        )
        static_density = self.density_activation(geo_feats[..., 0])

        has_timestamps = (
            "normed_timestamps" in data_dict or "lidar_normed_timestamps" in data_dict
        )
        if "scene_id" in data_dict:
            scene_id = data_dict["scene_id"]
        if len(self.dynamic_xyz_encoder) > 0 and has_timestamps:
            # forward dynamic branch
            if "normed_timestamps" in data_dict:
                normed_timestamps = data_dict["normed_timestamps"]
            elif "lidar_normed_timestamps" in data_dict:
                # we use `lidar_` prefix as an identifier to skip querying other heads
                normed_timestamps = data_dict["lidar_normed_timestamps"]
            dynamic_feats, dynamic_hash_encodings = self.forward_dynamic_hash(
                scene_id , normed_positions, normed_timestamps, return_hash_encodings=True
            )
            if len(self.flow_xyz_encoder) > 0:
                flow = self.forward_flow_hash(scene_id, normed_positions, normed_timestamps)
                forward_flow, backward_flow = flow[..., :3], flow[..., 3:]
                results_dict["forward_flow"] = forward_flow
                results_dict["backward_flow"] = backward_flow
                temporal_aggregation_results = self.temporal_aggregation(
                    scene_id,
                    positions,
                    normed_timestamps,
                    forward_flow,
                    backward_flow,
                    dynamic_feats,
                )
                # overwrite dynamic feats using temporal aggregation results
                dynamic_feats = temporal_aggregation_results["dynamic_feats"]
                # to be studied
                temporal_aggregation_results[
                    "current_dynamic_hash_encodings"
                ] = dynamic_hash_encodings
                results_dict.update(temporal_aggregation_results)
            (dynamic_geo_feats, dynamic_semantic_feats,) = torch.split(
                dynamic_feats,
                [self.geometry_feature_dim, self.semantic_feature_dim],
                dim=-1,
            )
            dynamic_density = self.density_activation(dynamic_geo_feats[..., 0])
            # blend static and dynamic density to get the final density
            density = static_density + dynamic_density
            results_dict.update(
                {
                    "density": density,
                    "static_density": static_density,
                    "dynamic_density": dynamic_density,
                }
            )
            if return_density_only:
                # skip querying other heads
                return results_dict

            if directions is not None:
                rgb_results = self.query_rgb(
                    scene_id, directions, geo_feats, dynamic_geo_feats, data_dict=data_dict
                )
                results_dict["dynamic_rgb"] = rgb_results["dynamic_rgb"]
                results_dict["static_rgb"] = rgb_results["rgb"]
                if combine_static_dynamic:
                    static_ratio = static_density / (density + 1e-6)
                    dynamic_ratio = dynamic_density / (density + 1e-6)
                    results_dict["rgb"] = (
                        static_ratio[..., None] * results_dict["static_rgb"]
                        + dynamic_ratio[..., None] * results_dict["dynamic_rgb"]
                    )
            if self.enable_shadow_head:
                shadow_ratio = self.shadow_head(dynamic_geo_feats)
                results_dict["shadow_ratio"] = shadow_ratio
                if combine_static_dynamic and "rgb" in results_dict:
                    results_dict["rgb"] = (
                        static_ratio[..., None]
                        * results_dict["rgb"]
                        * (1 - shadow_ratio)
                        + dynamic_ratio[..., None] * results_dict["dynamic_rgb"]
                    )
        else:
            # if no dynamic branch, use static density
            results_dict["density"] = static_density
            if return_density_only:
                # skip querying other heads
                return results_dict
            if directions is not None:
                rgb_results = self.query_rgb(scene_id, directions, geo_feats, data_dict=data_dict)
                results_dict["rgb"] = rgb_results["rgb"]

        if self.enable_feature_head and query_feature_head:
            if self.enable_learnable_pe and query_pe_head:
                learnable_pe_map = (
                    F.grid_sample(
                        self.learnable_pe_map,
                        # assume pixel coords have been normalize to [-1, 1]
                        data_dict["pixel_coords"].reshape(1, 1, -1, 2) * 2 - 1,
                        align_corners=False,  # didn't test with True
                        mode="bilinear",  # didn't test with other modes
                    )
                    .squeeze(2)
                    .squeeze(0)
                    .permute(1, 0)
                )
                dino_pe = self.pe_head(learnable_pe_map)
                results_dict["dino_pe"] = dino_pe
            dino_feats = self.dino_head(semantic_feats)

            if len(self.dynamic_xyz_encoder) > 0 and has_timestamps:
                dynamic_dino_feats = self.dino_head(dynamic_semantic_feats)
                results_dict["static_dino_feat"] = dino_feats
                results_dict["dynamic_dino_feat"] = dynamic_dino_feats
                if combine_static_dynamic:
                    static_ratio = static_density / (density + 1e-6)
                    dynamic_ratio = dynamic_density / (density + 1e-6)
                    results_dict["dino_feat"] = (
                        static_ratio[..., None] * dino_feats
                        + dynamic_ratio[..., None] * dynamic_dino_feats
                    )
            else:
                results_dict["dino_feat"] = dino_feats

        # query sky if not in lidar mode
        if (
            self.enable_sky_head
            and "lidar_origin" not in data_dict
            and directions is not None
        ):
            directions = directions[:, 0]
            reduced_data_dict = {k: v[:, 0] for k, v in data_dict.items() if k not in ["scene_id"]}
            sky_results = self.query_sky(scene_id, directions, data_dict=reduced_data_dict)
            results_dict.update(sky_results)

        return results_dict

    def temporal_aggregation(
        self,
        scene_id: str,
        positions: Tensor,  # current world coordinates
        normed_timestamps: Tensor,  # current normalized timestamps
        forward_flow: Tensor,
        backward_flow: Tensor,
        dynamic_feats: Tensor,
    ) -> Tensor:
        """
        temporal aggregation for dynamic features
        Eq. (8) in the emernerf paper
        """
        if normed_timestamps.shape[-1] != 1:
            normed_timestamps = normed_timestamps.unsqueeze(-1)
        if self.training:
            noise = torch.rand_like(forward_flow)[..., 0:1]
        else:
            noise = torch.ones_like(forward_flow)[..., 0:1]
        # forward and backward warped positions
        forward_warped_positions = self.contract_points(
            positions + forward_flow * noise
        )
        backward_warped_positions = self.contract_points(
            positions + backward_flow * noise
        )
        # forward and backward warped timestamps
        forward_warped_time = torch.clamp(
            normed_timestamps + self.time_diff[scene_id] * noise, 0, 1.0
        )
        backward_warped_time = torch.clamp(
            normed_timestamps - self.time_diff[scene_id] * noise, 0, 1.0
        )
        (
            forward_dynamic_feats,
            forward_dynamic_hash_encodings,
        ) = self.forward_dynamic_hash(
            scene_id,
            forward_warped_positions,
            forward_warped_time,
            return_hash_encodings=True,
        )
        (
            backward_dynamic_feats,
            backward_dynamic_hash_encodings,
        ) = self.forward_dynamic_hash(
            scene_id,
            backward_warped_positions,
            backward_warped_time,
            return_hash_encodings=True,
        )
        forward_pred_flow = self.forward_flow_hash(
            scene_id,
            forward_warped_positions,
            forward_warped_time,
        )
        backward_pred_flow = self.forward_flow_hash(
            scene_id,
            backward_warped_positions,
            backward_warped_time,
        )
        # simple weighted sum
        aggregated_dynamic_feats = (
            dynamic_feats + 0.5 * forward_dynamic_feats + 0.5 * backward_dynamic_feats
        ) / 2.0
        return {
            "dynamic_feats": aggregated_dynamic_feats,
            "forward_pred_backward_flow": forward_pred_flow[..., 3:],
            "backward_pred_forward_flow": backward_pred_flow[..., :3],
            # to be studied
            "forward_dynamic_hash_encodings": forward_dynamic_hash_encodings,
            "backward_dynamic_hash_encodings": backward_dynamic_hash_encodings,
        }

    def query_rgb(
        self,
        scene_id,
        directions: Tensor,
        geo_feats: Tensor,
        dynamic_geo_feats: Tensor = None,
        data_dict: Dict[str, Tensor] = None,
    ) -> Tensor:
        directions = (directions + 1.0) / 2.0  # do we need this?
        h = self.direction_encoding(directions.reshape(-1, directions.shape[-1])).view(
            *directions.shape[:-1], -1
        )
        if self.enable_cam_embedding or self.enable_img_embedding:
            if "cam_idx" in data_dict and self.enable_cam_embedding:
                appearance_embedding = self.appearance_embedding(data_dict["cam_idx"])
            elif "img_idx" in data_dict and self.enable_img_embedding:
                appearance_embedding = self.appearance_embedding[scene_id](data_dict["img_idx"])
            else:
                # use mean appearance embedding
                # print("using mean appearance embedding")
                appearance_embedding = torch.ones(
                    (*directions.shape[:-1], self.appearance_embedding_dim),
                    device=directions.device,
                ) * self.appearance_embedding.weight.mean(dim=0)
            h = torch.cat([h, appearance_embedding], dim=-1)

        rgb = self.rgb_head(torch.cat([h, geo_feats], dim=-1))
        rgb = F.sigmoid(rgb)
        results = {"rgb": rgb}

        if len(self.dynamic_xyz_encoder) > 0:
            assert (
                dynamic_geo_feats is not None
            ), "Dynamic geometry features are not provided."
            dynamic_rgb = self.rgb_head(torch.cat([h, dynamic_geo_feats], dim=-1))
            dynamic_rgb = F.sigmoid(dynamic_rgb)
            results["dynamic_rgb"] = dynamic_rgb
        return results

    def query_sky(
        self, scene_id, directions: Tensor, data_dict: Dict[str, Tensor] = None
    ) -> Dict[str, Tensor]:
        if len(directions.shape) == 2:
            dd = self.direction_encoding(directions).to(directions)
        else:
            dd = self.direction_encoding(directions[:, 0]).to(directions)
        if self.enable_cam_embedding or self.enable_img_embedding:
            # optionally add appearance embedding
            if "cam_idx" in data_dict and self.enable_cam_embedding:
                appearance_embedding = self.appearance_embedding[scene_id](data_dict["cam_idx"])
            elif "img_idx" in data_dict and self.enable_img_embedding:
                appearance_embedding = self.appearance_embedding[scene_id](data_dict["img_idx"])
            else:
                # use mean appearance embedding
                appearance_embedding = torch.ones(
                    (*directions.shape[:-1], self.appearance_embedding_dim),
                    device=directions.device,
                ) * self.appearance_embedding.weight.mean(dim=0)
            dd = torch.cat([dd, appearance_embedding], dim=-1)
        rgb_sky = self.sky_head(dd).to(directions)
        rgb_sky = F.sigmoid(rgb_sky)
        results = {"rgb_sky": rgb_sky}
        if self.enable_feature_head:
            self.dino_sky_head(dd).to(directions)
            results["dino_sky_feat"] = self.dino_sky_head(dd).to(directions)
        return results

    def query_flow(
        self, positions: Tensor, normed_timestamps: Tensor, query_density: bool = True
    ) -> Dict[str, Tensor]:
        """
        query flow field
        """
        normed_positions = self.contract_points(positions)
        flow = self.forward_flow_hash(normed_positions, normed_timestamps)
        results = {
            "forward_flow": flow[..., :3],
            "backward_flow": flow[..., 3:],
        }
        if query_density:
            # it's important to filter valid flows based on a dynamic density threshold.
            # flows are valid only if they are on dynamic points.
            dynamic_feats = self.forward_dynamic_hash(
                normed_positions, normed_timestamps
            )
            (dynamic_geo_feats, _,) = torch.split(
                dynamic_feats,
                [self.geometry_feature_dim, self.semantic_feature_dim],
                dim=-1,
            )
            dynamic_density = self.density_activation(dynamic_geo_feats[..., 0])
            results["dynamic_density"] = dynamic_density
        return results

    def query_attributes(
        self,
        positions: Tensor,
        normed_timestamps: Tensor = None,
        query_feature_head: bool = True,
    ):
        """
        query attributes (density, dino features, etc.)
        """
        results_dict = {}
        encoded_features, normed_positions = self.forward_static_hash(positions)
        geo_feats, semantic_feats = torch.split(
            encoded_features,
            [self.geometry_feature_dim, self.semantic_feature_dim],
            dim=-1,
        )
        static_density = self.density_activation(geo_feats[..., 0])
        if len(self.dynamic_xyz_encoder) > 0 and normed_timestamps is not None:
            dynamic_feats, dynamic_hash_encodings = self.forward_dynamic_hash(
                normed_positions, normed_timestamps, return_hash_encodings=True
            )
            if len(self.flow_xyz_encoder) > 0:
                flow = self.forward_flow_hash(normed_positions, normed_timestamps)
                forward_flow = flow[..., :3]
                backward_flow = flow[..., 3:]
                results_dict["forward_flow"] = forward_flow
                results_dict["backward_flow"] = backward_flow
                temporal_aggregation_results = self.temporal_aggregation(
                    positions,
                    normed_timestamps,
                    forward_flow,
                    backward_flow,
                    dynamic_feats,
                )
                dynamic_feats = temporal_aggregation_results["dynamic_feats"]
                temporal_aggregation_results[
                    "current_dynamic_hash_encodings"
                ] = dynamic_hash_encodings
                results_dict.update(temporal_aggregation_results)

            (dynamic_geo_feats, dynamic_semantic_feats,) = torch.split(
                dynamic_feats,
                [self.geometry_feature_dim, self.semantic_feature_dim],
                dim=-1,
            )
            dynamic_density = self.density_activation(dynamic_geo_feats[..., 0])
            density = static_density + dynamic_density
            results_dict.update(
                {
                    "density": density,
                    "static_density": static_density,
                    "dynamic_density": dynamic_density,
                    # "occupancy": occupancy,
                }
            )
        else:
            results_dict["density"] = static_density
        if self.enable_feature_head and query_feature_head:
            # query on demand
            dino_feats = self.dino_head(semantic_feats)
            if len(self.dynamic_xyz_encoder) > 0 and normed_timestamps is not None:
                dynamic_dino_feats = self.dino_head(dynamic_semantic_feats)
                results_dict["static_dino_feat"] = dino_feats
                results_dict["dynamic_dino_feat"] = dynamic_dino_feats
                results_dict["dino_feat"] = (
                    static_density.unsqueeze(-1) * dino_feats
                    + dynamic_density.unsqueeze(-1) * dynamic_dino_feats
                ) / (density.unsqueeze(-1) + 1e-6)
            else:
                results_dict["dino_feat"] = dino_feats
        return results_dict


def build_composed_radiance_field_from_cfg(cfg, verbose=True) -> ComposedRadianceField:
    xyz_encoder = build_xyz_encoder_from_cfg(cfg.xyz_encoder, verbose=verbose)
    dynamic_xyz_encoders = nn.ParameterDict()
    flow_xyz_encoders = nn.ParameterDict()
    if cfg.head.enable_dynamic_branch:
        for scene_id in cfg.scene_ids:
            dynamic_xyz_encoders[scene_id] = build_xyz_encoder_from_cfg(
                cfg.dynamic_xyz_encoder, verbose=verbose
            )

    if cfg.head.enable_flow_branch:
        for scene_id in cfg.scene_ids:
            flow_xyz_encoders[scene_id] = HashEncoder(
                n_input_dims=4,
                n_levels=10,
                base_resolution=16,
                max_resolution=4096,
                log2_hashmap_size=18,
                n_features_per_level=4,
            )
        
    return ComposedRadianceField(
        xyz_encoder=xyz_encoder,
        dynamic_xyz_encoders=dynamic_xyz_encoders,
        flow_xyz_encoders=flow_xyz_encoders,
        unbounded=cfg.unbounded,
        num_cams=cfg.num_cams,
        geometry_feature_dim=cfg.neck.geometry_feature_dim,
        base_mlp_layer_width=cfg.neck.base_mlp_layer_width,
        head_mlp_layer_width=cfg.head.head_mlp_layer_width,
        enable_cam_embedding=cfg.head.enable_cam_embedding,
        enable_img_embedding=cfg.head.enable_img_embedding,
        appearance_embedding_dim=cfg.head.appearance_embedding_dim,
        enable_sky_head=cfg.head.enable_sky_head,
        enable_feature_head=cfg.head.enable_feature_head,
        semantic_feature_dim=cfg.neck.semantic_feature_dim,
        feature_mlp_layer_width=cfg.head.feature_mlp_layer_width,
        feature_embedding_dim=cfg.head.feature_embedding_dim,
        enable_shadow_head=cfg.head.enable_shadow_head,
        num_train_timesteps=cfg.num_train_timesteps,  # placeholder
        interpolate_xyz_encoding=cfg.head.interpolate_xyz_encoding,
        enable_learnable_pe=cfg.head.enable_learnable_pe,
        enable_temporal_interpolation=cfg.head.enable_temporal_interpolation,
    )
