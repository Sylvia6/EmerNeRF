import cv2
import plus_general.utils.matrix_utils as matrix_utils
import numpy as np
from plus_general.utils.flow_utils import resize_flow


class _DISFlow(object):
    def __init__(self):
        self.flow_calculator = cv2.DISOpticalFlow.create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM)

    def forward(self, im0, im1):
        im0_gray = cv2.cvtColor(im0, cv2.COLOR_BGR2GRAY)
        im1_gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
        flow = self.flow_calculator.calc(im0_gray, im1_gray, None)
        return flow


class DisFlow(object):
    def __init__(self, p_matrix, imu_to_cam, imu_height=0.52, dst_size=(512, 256)):
        self.optical_flow_calc = _DISFlow()
        self.p_matrix = p_matrix
        self.imu_to_cam = imu_to_cam
        self.imu_height = imu_height
        self.image_to_imu = matrix_utils.get_image_to_imu(self.p_matrix, self.imu_to_cam, self.imu_height)
        self.dst_size = dst_size

    def forward(self, im0, im1, pose0, pose1, warp=True, src_image_warp_matrix=None, reverse_flow=False, add_offset=True):
        src_size = (im0.shape[1], im1.shape[0])
        im0 = cv2.resize(im0, self.dst_size)
        im1 = cv2.resize(im1, self.dst_size)
        if src_image_warp_matrix is None:
            src_image_warp_matrix = matrix_utils.pose_image_perspective_matrix(pose0, pose1,
                                                                               self.image_to_imu, self.p_matrix,
                                                                               self.imu_to_cam, self.imu_height)
        resize_image_warp_matrix = matrix_utils.resize_transform_matrix(src_image_warp_matrix,
                                                                        src_size, self.dst_size)
        if warp:
            im0_warp = cv2.warpPerspective(im0, resize_image_warp_matrix,
                                           (im0.shape[1], im0.shape[0]))
        else:
            im0_warp = im0

        if not reverse_flow:
            flow = self.optical_flow_calc.forward(im0_warp, im1)
        else:
            flow = self.optical_flow_calc.forward(im1, im0_warp)

        if add_offset and warp:
            flow = cv2.warpPerspective(flow, np.linalg.inv(resize_image_warp_matrix),
                                       (im0_warp.shape[1], im0_warp.shape[0]))
            h, w = flow.shape[:2]
            x, y = np.meshgrid(np.arange(w), np.arange(h))
            origin = np.stack((x, y)).reshape(2, -1).astype('double')
            origin_homo = np.concatenate((origin + 0.5, np.ones((1, origin.shape[1]))), 0)
            after_homo = np.matmul(resize_image_warp_matrix, origin_homo)
            after_homo = after_homo / after_homo[-1]
            after = after_homo[:2] - 0.5
            offset = after - origin
            offset = offset[:2]
            offset = offset.reshape((2, flow.shape[0], flow.shape[1]))
            offset = offset.transpose((1, 2, 0))
            flow = flow + offset

        flow = resize_flow(flow, src_size[0], src_size[1])
        return flow
