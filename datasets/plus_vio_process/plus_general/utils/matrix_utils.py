import numpy as np
import cv2
from scipy.spatial.transform import Rotation


def image_scale_matrix(s0, s1):
    result = np.eye(3, 3)
    result[0, 0] = s0
    result[1, 1] = s1
    return result


def get_distance_image_y_using_calibration(distance, p_matrix, imu_to_cam, imu_height):
    road_to_imu = np.mat([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -imu_height],
        [0, 0, 0, 1]
    ])
    road_to_image = p_matrix * imu_to_cam * road_to_imu
    road_pt = np.array([distance, 0, 0, 1]).reshape((4, 1))
    image_pt = road_to_image * road_pt
    image_pt = image_pt / image_pt[2]
    return image_pt[1]


def get_homography(p1, p2, imu2cam, imu_height):
    points = [
        [10, -2, -imu_height, 1],
        [10, 2, -imu_height, 1],
        [50, -2, -imu_height, 1],
        [50, 2, -imu_height, 1]
    ]

    road_points = np.array(points)

    im1_points = p1 * imu2cam * road_points.transpose((1, 0))
    im1_points = im1_points/im1_points[-1, :]
    im1_points = np.array(im1_points).transpose((1, 0))[:, :2]
    im2_points = p2 * imu2cam * road_points.transpose((1, 0))
    im2_points = im2_points/im2_points[-1, :]
    im2_points = np.array(im2_points).transpose((1, 0))[:, :2]
    M, _ = cv2.findHomography(im2_points, im1_points, 0)
    return M


def get_image_to_imu(p_matrix, imu_to_cam, imu_height):
    refined_imu_to_image = np.matmul(np.array(p_matrix), np.array(imu_to_cam))
    refined_imu_to_image_ground = np.array(refined_imu_to_image[:3, :3])
    for i in range(3):
        refined_imu_to_image_ground[i, 2] = refined_imu_to_image[i, 3] + refined_imu_to_image[i, 2]*(-imu_height)
    mat = np.mat(refined_imu_to_image_ground).I
    return np.array(mat)


def get_imu_to_image(p_matrix, imu_to_cam):
    return np.matmul(np.array(p_matrix), np.array(imu_to_cam))


def relative_matrix(matrix_0, matrix_1):
    return np.matmul(np.linalg.inv(matrix_0), matrix_1)


def matrix_from_pose(pose):
    x, y, z, roll, pitch, yaw = pose
    rot = Rotation.from_euler('XYZ', [roll, pitch, yaw])
    result = np.eye(4)
    result[:3, :3] = rot.as_matrix()
    result[0, -1] = x
    result[1, -1] = y
    result[2, -1] = z
    return result


def pose_from_matrix(matrix):
    rot = Rotation.from_matrix(matrix[:3, :3])
    rads = rot.as_euler('XYZ')
    return (matrix[0, -1], matrix[1, -1], matrix[2, -1], rads[0], rads[1], rads[2])


def relative_pose(base, dst):
    m1 = matrix_from_pose(base)
    m2 = matrix_from_pose(dst)
    ret = np.matmul(np.linalg.inv(m1), m2)
    return pose_from_matrix(ret)


def estimate_imu_rt_change_image_perspective(image_to_imu, p_matrix, imu_to_cam, imu_height, RT):
    new_image_to_imu_matrix = np.zeros((4, 3))
    new_image_to_imu_matrix[:2] = image_to_imu[:2]
    new_image_to_imu_matrix[3] = image_to_imu[2]
    new_image_to_imu_matrix[2] = new_image_to_imu_matrix[3] * -imu_height
    after_rt = np.matmul(RT, new_image_to_imu_matrix)
    imu_to_image = np.matmul(np.array(p_matrix), np.array(imu_to_cam))
    imu_to_image = np.array(imu_to_image)
    result = np.matmul(imu_to_image, after_rt)
    return result


def pose_image_perspective_matrix(pose0, pose1, image_to_imu, p_matrix, imu_to_cam, imu_height):
    rt = matrix_from_pose(relative_pose(pose1, pose0))
    return estimate_imu_rt_change_image_perspective(image_to_imu, p_matrix, imu_to_cam, imu_height, rt)


def resize_transform_matrix(matrix, src_size=None, dst_size=None, s0=None, s1=None):
    scale_matrix = np.eye(3)
    if src_size is not None:
        s0 = dst_size[0] / float(src_size[0])
        s1 = dst_size[1] / float(src_size[1])

    scale_matrix[0, 0] = s0
    scale_matrix[1, 1] = s1

    dst_matrix = np.eye(3)
    dst_matrix = np.matmul(np.linalg.inv(scale_matrix), dst_matrix)
    dst_matrix = np.matmul(matrix, dst_matrix)
    dst_matrix = np.matmul(scale_matrix, dst_matrix)
    return dst_matrix


def homo_mul(image_points, matrix, image=True, hnorm=True):
    image_points = np.concatenate((image_points, np.ones((image_points.shape[0], 1))), 1)
    if image:
        image_points[:, 0] += 0.5
        image_points[:, 1] += 0.5
    image_points = image_points.transpose((1, 0))  # 3*n
    transform_points = np.matmul(matrix, image_points)  # 3*n
    if hnorm:
        transform_points[:-1, :] /= transform_points[-1]
        transform_points = transform_points.transpose((1, 0))
        if image:
            transform_points[:, 0] -= 0.5
            transform_points[:, 1] -= 0.5
        return transform_points[:, :-1]
    else:
        return transform_points.transpose((1, 0))


def get_pose_rt_matrix(pose_0, pose_1):
    m1 = matrix_from_pose(pose_0)
    m2 = matrix_from_pose(pose_1)
    return np.matmul(np.linalg.inv(m1), m2)


def get_image_to_imu_image_matrix(p_matrix, imu_to_cam, imu_height, dst_w, dst_h, bev_max_x, bev_max_y, rear=False):
    image_to_imu = get_image_to_imu(p_matrix, imu_to_cam, imu_height)
    # test_points = np.array([(480, 440), (480, 540)])
    # imu_points = homo_mul(test_points, image_to_imu)
    scale_matrix = np.identity(3, dtype='double')
    scale_matrix[0, 0] = 0
    scale_matrix[0, 1] = -dst_w / float(2 * bev_max_y)
    scale_matrix[0, 2] = dst_w / 2.0
    scale_matrix[1, 1] = 0
    scale_matrix[1, 0] = -dst_h / float(bev_max_x)
    if not rear:
        scale_matrix[1, 2] = dst_h
    else:
        pass
    image_to_imu_image_matrix = np.matmul(scale_matrix, image_to_imu)
    return image_to_imu_image_matrix


def get_image_to_imu_roi_image_matrix(p_matrix, imu_to_cam, imu_height, dst_h, dst_w, imu_roi, rear=False):
    image_to_imu = get_image_to_imu(p_matrix, imu_to_cam, imu_height)
    # test_points = np.array([(480, 440), (480, 540)])
    # imu_points = homo_mul(test_points, image_to_imu)

    x0, x1, y0, y1 = imu_roi

    t_matrix = np.identity(3, dtype='double')
    t_matrix[0, 2] = -x0
    t_matrix[1, 2] = -y0

    scale_matrix = np.identity(3, dtype='double')
    # scale_matrix[0, 0] = 0
    # scale_matrix[0, 1] = -dst_w / float(2 * bev_max_y)
    # scale_matrix[0, 2] = dst_w / 2.0
    # scale_matrix[1, 1] = 0
    # scale_matrix[1, 0] = -dst_h / float(bev_max_x)
    scale_matrix[0, 0] = dst_w / float(x1-x0)
    scale_matrix[1, 1] = dst_h / float(y1-y0)

    imu_to_imu_image_matrix = np.matmul(scale_matrix, t_matrix)
    # if not rear:
    #     scale_matrix[1, 2] = dst_h
    # else:
    #     pass
    image_to_imu_image_matrix = np.matmul(imu_to_imu_image_matrix, image_to_imu)
    return image_to_imu_image_matrix, imu_to_imu_image_matrix


def rigid_transform_3D(A, B):
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    # if num_rows != 3:
    #     raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")
    #
    # num_rows, num_cols = B.shape
    # if num_rows != 3:
    #     raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am.dot(np.transpose(Bm))

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = (Vt.T).dot(U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        # print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = (Vt.T).dot(U.T)

    t = (-R).dot(centroid_A) + centroid_B

    return R, t


def get_road_homography(p1, p2, imu2cam, imu_height):
    image1_to_imu = get_image_to_imu(p1, imu2cam, imu_height)
    image2_to_imu = get_image_to_imu(p2, imu2cam, imu_height)
    image2_to_image1 = np.matmul(np.linalg.inv(image1_to_imu), image2_to_imu)
    M = image2_to_image1
    M[1, 0] = 0
    M[1, 1] = 1
    M[1, 2] = 0
    M[2, 0] = 0
    M[2, 1] = 0
    M[2, 2] = 1
    return M