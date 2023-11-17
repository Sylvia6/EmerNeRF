import numpy as np
import cv2
import torch
import torch.nn as nn
import os
from .unimatch import UniMatch
import torch.nn.functional as F
# from torchvision.transforms.functional import hflip
from .data import stereo_transforms as stereo_transforms
# from .geometry import forward_backward_consistency_check

MODEL_ROOT="/mnt/intel/jupyterhub/siyuan.yang/models/vio"
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


def hflip(img):
    return img.flip(-1)


@torch.no_grad()
def inference_stereo(model,
                     left,
                     right,
                     padding_factor=16,
                     inference_size=None,
                     attn_type=None,
                     attn_splits_list=None,
                     corr_radius_list=None,
                     prop_radius_list=None,
                     num_reg_refine=1,
                     pred_bidir_disp=False,
                     pred_right_disp=False,
                     device='cuda'
                     ):
    model.eval()

    val_transform_list = [stereo_transforms.ToTensor(),
                          stereo_transforms.Normalize(mean=IMAGENET_MEAN, std=IMAGENET_STD)
                          ]

    val_transform = stereo_transforms.Compose(val_transform_list)

    fixed_inference_size = inference_size

    sample = {'left': left.astype('float32'), 'right': right.astype('float32')}
    sample = val_transform(sample)
    left = sample['left'].to(device).unsqueeze(0)  # [1, 3, H, W]
    right = sample['right'].to(device).unsqueeze(0)  # [1, 3, H, W]

    nearest_size = [int(np.ceil(left.size(-2) / padding_factor)) * padding_factor,
                    int(np.ceil(left.size(-1) / padding_factor)) * padding_factor]

    # resize to nearest size or specified size
    inference_size = nearest_size if fixed_inference_size is None else fixed_inference_size

    ori_size = left.shape[-2:]
    if inference_size[0] != ori_size[0] or inference_size[1] != ori_size[1]:
        left = F.interpolate(left, size=inference_size,
                             mode='bilinear',
                             align_corners=True)
        right = F.interpolate(right, size=inference_size,
                              mode='bilinear',
                              align_corners=True)

    with torch.no_grad():
        if pred_bidir_disp:
            new_left, new_right = hflip(right), hflip(left)
            left = torch.cat((left, new_left), dim=0)
            right = torch.cat((right, new_right), dim=0)

        if pred_right_disp:
            left, right = hflip(right), hflip(left)

        pred_disp = model(left, right,
                          attn_type=attn_type,
                          attn_splits_list=attn_splits_list,
                          corr_radius_list=corr_radius_list,
                          prop_radius_list=prop_radius_list,
                          num_reg_refine=num_reg_refine,
                          task='stereo',
                          )['flow_preds'][-1]  # [1, H, W]
    pred_disp_before_resize = pred_disp
    if inference_size[0] != ori_size[0] or inference_size[1] != ori_size[1]:
        # resize back
        pred_disp = F.interpolate(pred_disp.unsqueeze(1), size=ori_size,
                                  mode='bilinear',
                                  align_corners=True).squeeze(1)  # [1, H, W]
        pred_disp = pred_disp * ori_size[-1] / float(inference_size[-1])

    if pred_right_disp:
        pred_disp = hflip(pred_disp)

    disp = pred_disp[0].cpu().numpy()

    if pred_bidir_disp:
        assert pred_disp.size(0) == 2  # [2, H, W]
        disp = hflip(pred_disp[1]).cpu().numpy()
    return disp, pred_disp_before_resize[0].cpu().numpy()


@torch.no_grad()
def inference_flow(model,
                   image1,
                   image2,
                   padding_factor=8,
                   inference_size=None,
                   attn_type='swin',
                   attn_splits_list=None,
                   corr_radius_list=None,
                   prop_radius_list=None,
                   num_reg_refine=1,
                   pred_bidir_flow=False,
                   pred_bwd_flow=False,
                   fwd_bwd_consistency_check=False,
                   ):
    """ Inference on a directory or a video """
    model.eval()

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    if fwd_bwd_consistency_check:
        assert pred_bidir_flow

    fixed_inference_size = inference_size
    transpose_img = False

    vis_flow_preds = []
    ori_imgs = []

    image1 = np.array(image1).astype(np.uint8)
    image2 = np.array(image2).astype(np.uint8)

    if len(image1.shape) == 2:  # gray image
        image1 = np.tile(image1[..., None], (1, 1, 3))
        image2 = np.tile(image2[..., None], (1, 1, 3))
    else:
        image1 = image1[..., :3]
        image2 = image2[..., :3]

    image1 = torch.from_numpy(image1).permute(2, 0, 1).float().unsqueeze(0).to(device)
    image2 = torch.from_numpy(image2).permute(2, 0, 1).float().unsqueeze(0).to(device)

    # the model is trained with size: width > height
    if image1.size(-2) > image1.size(-1):
        image1 = torch.transpose(image1, -2, -1)
        image2 = torch.transpose(image2, -2, -1)
        transpose_img = True

    nearest_size = [int(np.ceil(image1.size(-2) / padding_factor)) * padding_factor,
                    int(np.ceil(image1.size(-1) / padding_factor)) * padding_factor]

    # resize to nearest size or specified size
    inference_size = nearest_size if fixed_inference_size is None else fixed_inference_size

    assert isinstance(inference_size, list) or isinstance(inference_size, tuple)
    ori_size = image1.shape[-2:]

    # resize before inference
    if inference_size[0] != ori_size[0] or inference_size[1] != ori_size[1]:
        image1 = F.interpolate(image1, size=inference_size, mode='bilinear',
                               align_corners=True)
        image2 = F.interpolate(image2, size=inference_size, mode='bilinear',
                               align_corners=True)

    if pred_bwd_flow:
        image1, image2 = image2, image1

    results_dict = model(image1, image2,
                         attn_type=attn_type,
                         attn_splits_list=attn_splits_list,
                         corr_radius_list=corr_radius_list,
                         prop_radius_list=prop_radius_list,
                         num_reg_refine=num_reg_refine,
                         task='flow',
                         pred_bidir_flow=pred_bidir_flow,
                         )

    flow_pr = results_dict['flow_preds'][-1]  # [B, 2, H, W]

    # resize back
    if inference_size[0] != ori_size[0] or inference_size[1] != ori_size[1]:
        flow_pr = F.interpolate(flow_pr, size=ori_size, mode='bilinear',
                                align_corners=True)
        flow_pr[:, 0] = flow_pr[:, 0] * ori_size[-1] / inference_size[-1]
        flow_pr[:, 1] = flow_pr[:, 1] * ori_size[-2] / inference_size[-2]

    if transpose_img:
        flow_pr = torch.transpose(flow_pr, -2, -1)

    flow = flow_pr[0].permute(1, 2, 0).cpu().numpy()  # [H, W, 2]

    # also predict backward flow
    # if pred_bidir_flow:
    #     assert flow_pr.size(0) == 2  # [2, H, W, 2]
    #     flow_bwd = flow_pr[1].permute(1, 2, 0).cpu().numpy()  # [H, W, 2]
    #
    #     # forward-backward consistency check
    #     # occlusion is 1
    #     if fwd_bwd_consistency_check:
    #         fwd_occ, bwd_occ = forward_backward_consistency_check(flow_pr[:1], flow_pr[1:])  # [1, H, W] float

    return flow


class Inferencer(object):
    def __init__(self, task, inference_size):
        assert task in ['stereo', 'flow']
        device = 'cuda'
        self.num_head = 1
        self.feature_channels = 128
        self.ffn_dim_expansion = 4
        self.num_transformer_layers = 6
        self.pretrain_path = ""
        self.attn_type = None
        self.task = task
        self.inference_size = inference_size
        if task == 'stereo':
            self.padding_factor = 32
            self.upsample_factor = 4
            self.num_scales = 2
            self.attn_type = "self_swin2d_cross_swin1d"
            self.attn_splits_list = [2, 8]
            self.corr_radius_list = [-1, 4]
            self.prop_radius_list = [-1, 1]
            self.reg_refine = True
            self.num_reg_refine = 3
            self.pred_bidir_disp = False
            self.pred_right_disp = False
            resume = os.path.join(MODEL_ROOT,
                                  "gmstereo-scale2-regrefine3-resumeflowthings-mixdata-train320x640-ft640x960-e4e291fd.pth_new.pth")
        elif task == 'flow':
            self.padding_factor = 32
            self.upsample_factor = 4
            self.num_scales = 2
            self.attn_type = "swin"
            self.attn_splits_list = [2, 8]
            self.corr_radius_list = [-1, 4]
            self.prop_radius_list = [-1, 1]
            self.reg_refine = True
            self.num_reg_refine = 6
            self.pred_bidir_flow = False
            self.pred_bwd_flow = False
            self.fwd_bwd_check = False
            resume = os.path.join(MODEL_ROOT,
                                  "gmflow-scale2-regrefine6-mixdata-train320x576-4e7b215d.pth_new.pth")
            # self.padding_factor = 16
            # self.upsample_factor = 8
            # self.num_scales = 1
            # self.attn_type = "swin"
            # self.attn_splits_list = [2]
            # self.corr_radius_list = [-1]
            # self.prop_radius_list = [-1]
            # self.reg_refine = False
            # self.num_reg_refine = 1
            # self.pred_bidir_flow = False
            # self.pred_bwd_flow = False
            # self.fwd_bwd_check = False
            # resume = os.path.join(folder,
            #                       "gmflow-scale1-mixdata-train320x576-4c3a6e9a.pth")
        else:
            raise NotImplementedError

        self.model = UniMatch(feature_channels=self.feature_channels,
                         num_scales=self.num_scales,
                         upsample_factor=self.upsample_factor,
                         num_head=self.num_head,
                         ffn_dim_expansion=self.ffn_dim_expansion,
                         num_transformer_layers=self.num_transformer_layers,
                         reg_refine=self.reg_refine,
                         task=task).to(device)

        checkpoint = torch.load(resume)
        self.model.load_state_dict(checkpoint['model'], strict=False)

    def forward(self, image1, image2):
        if self.task == 'stereo':
            return inference_stereo(self.model, image1, image2,
                             padding_factor=self.padding_factor,
                             inference_size=self.inference_size,
                             attn_type=self.attn_type,
                             attn_splits_list=self.attn_splits_list,
                             corr_radius_list=self.corr_radius_list,
                             prop_radius_list=self.prop_radius_list,
                             num_reg_refine=self.num_reg_refine,
                             pred_bidir_disp=self.pred_bidir_disp,
                             pred_right_disp=self.pred_right_disp,
                             )
        elif self.task == 'flow':
            return inference_flow(self.model, image1, image2,
                                  padding_factor=self.padding_factor,
                                  inference_size=self.inference_size,
                                  attn_type=self.attn_type,
                                  attn_splits_list=self.attn_splits_list,
                                  corr_radius_list=self.corr_radius_list,
                                  prop_radius_list=self.prop_radius_list,
                                  pred_bidir_flow=self.pred_bidir_flow,
                                  pred_bwd_flow=self.pred_bwd_flow,
                                  num_reg_refine=self.num_reg_refine,
                                  fwd_bwd_consistency_check=self.fwd_bwd_check,
                                  )
