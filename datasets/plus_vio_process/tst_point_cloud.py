
from plus_general.utils.point_cloud2 import read_points
from plus_general.utils.bag_handler import ImageBagHandler
import numpy as np
bag_path = ""
lidar_topic = "/rslidar_points"
other_topics = [lidar_topic]
bag_path = "/mnt/intel/jupyterhub/lu.li/plus_data/20230319T090808_pdb-l4e-b0007_6_871to900.db"

bag_handler = ImageBagHandler(bag_path, front_left=True, other_topics=other_topics)

for msg_dict in bag_handler.msg_generator(to_image=True):
    pc_msg = msg_dict[lidar_topic][0].message
    if lidar_topic == '/livox/lidar' or lidar_topic == '/rslidar_points' or lidar_topic == '/innovusion_lidar/iv_points':
        lidar_pts = np.array(list(read_points(pc_msg, field_names=['x', 'y', 'z', 'intensity'], skip_nans=True)), dtype=np.float32)
        lidar_pts[:,:3] = lidar_pts[:,:3] * 0.01
    else:
        lidar_pts = np.array(list(read_points(pc_msg, field_names=['x', 'y', 'z', 'intensity', 'ring'], skip_nans=True)), dtype=np.float32)

    print(lidar_pts)


