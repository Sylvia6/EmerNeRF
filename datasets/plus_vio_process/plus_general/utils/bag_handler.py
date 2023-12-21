import fastbag
# from cv_bridge import CvBridge
import numpy as np
import collections
from plus_general.utils.ros_utils import deserialize_compress_image
from plus_general.utils.ros.msg import Odometry
from plus_general.utils.ros import message_register


def module_exists(module_name):
    try:
        __import__(module_name)
    except ImportError:
        return False
    else:
        return True


if module_exists('tf'):
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
else:
    from transformations import euler_from_quaternion as euler_from_quaternion_
    from transformations import quaternion_from_euler as quaternion_from_euler_

    def euler_from_quaternion(quaternion, axes='sxyz'):
        new_quaternion = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]
        return euler_from_quaternion_(new_quaternion, axes)

    def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
        result = quaternion_from_euler_(ai, aj, ak, axes)
        return [result[1], result[2], result[3], result[0]]



BagMessage = collections.namedtuple('BagMessage', 'topic message timestamp')

possible_lane_topics = [
    '/front_left_camera/image_color/compressed',
    '/front_left_camera/image_raw/compressed',
    '/usb_cam_left/image_raw/compressed',
]

possible_left_topics = possible_lane_topics

possible_right_topics = [
    '/front_right_camera/image_color/compressed',
    '/front_right_camera/image_raw/compressed',
    '/usb_cam_left/image_raw/compressed',
]

possible_rear_left_topics = [
    "/rear_left_camera/image_color/compressed"
]

possible_rear_right_topics = [
    "/rear_right_camera/image_color/compressed"
]


possible_side_left_topics = [
    "/side_left_camera/image_color/compressed"
]

possible_side_right_topics = [
    "/side_right_camera/image_color/compressed"
]


class BaseBag(object):
    def __init__(self, bag_path):
        self.bag_path = bag_path

    def msg_generator(self, topics):
        raise NotImplementedError

    def check_topic_exists(self, topic):
        raise NotImplementedError

    def check_topics_exists(self, topics):
        raise NotImplementedError

    def get_first_exists_topic(self, topics):
        for topic in topics:
            if self.check_topics_exists([topic]):
                return topic
        return None

    def guess_lane_image_topic(self):
        for topic in possible_lane_topics:
            if self.check_topics_exists([topic]):
                return topic
        return None

    def guess_right_image_topic(self):
        for topic in possible_right_topics:
            if self.check_topics_exists([topic]):
                return topic
        return None

    def guess_left_image_topic(self):
        return self.guess_lane_image_topic()

    def guess_rear_left_image_topic(self):
        return self.get_first_exists_topic(possible_rear_left_topics)

    def guess_rear_right_image_topic(self):
        return self.get_first_exists_topic(possible_rear_right_topics)

    def guess_side_left_image_topic(self):
        return self.get_first_exists_topic(possible_side_left_topics)

    def guess_side_right_image_topic(self):
        return self.get_first_exists_topic(possible_side_right_topics)

    def close(self):
        pass


def get_bag(bag_path):
    if bag_path.endswith('.db'):
        return FastBag(bag_path)
    else:
        return RosBag(bag_path)


class RosBag(BaseBag):
    def __init__(self, bag_path):
        super(RosBag, self).__init__(bag_path)
        import rosbag
        self.reader = rosbag.Bag(bag_path)

    def msg_generator(self, topics, raw=False):
        for msg in self.reader.read_messages(topics=topics, raw=raw):
            yield msg

    def check_topic_exists(self, topic):
        topics = [c.topic for c in self.reader._get_connections()]
        for t in topics:
            if t == topic:
                return True
        return False

    def check_topics_exists(self, topics):
        exist_topics = set()
        topics = [c.topic for c in self.reader._get_connections()]
        for t in topics:
            exist_topics.add(t)

        for topic in topics:
            if topic not in exist_topics:
                return False
        return True


class FastBag(BaseBag):
    def __init__(self, bag_path):
        super(FastBag, self).__init__(bag_path)
        self.reader = fastbag.Reader(self.bag_path)
        self.reader.open()

    def check_topic_exists(self, topic):
        for k, v in self.reader.iter_topics():
            if k == topic:
                return True
        return False

    def check_topics_exists(self, topics):
        exist_topics = set()
        for k, v in self.reader.iter_topics():
            exist_topics.add(k)

        for topic in topics:
            if topic not in exist_topics:
                return False
        return True

    def msg_generator(self, topics, raw=False):
        for topic, msg, ts in self.reader.iter_messages(topics=topics, ros_time=not raw, raw=raw):
            yield BagMessage(topic, msg, ts)

    def close(self):
        self.reader.close()


# def get_msg_timestamp(msg):
#     return msg.header.stamp


def msg_to_timestamp(msg):
    return msg.header.stamp


def msg_to_odom(odom_list, timestamp):
    def linear_interpolation(before, after, proportion):
        return before + proportion * (after - before)

    def wrapToPi(a):
        return a

    def slerp(before, after, proportion):
        return wrapToPi(before + proportion * wrapToPi(after - before))

    def getPoseFromOdom(odom0, odom1, proportion):
        x = linear_interpolation(odom0.position.x, odom1.position.x, proportion)
        y = linear_interpolation(odom0.position.y, odom1.position.y, proportion)
        z = linear_interpolation(odom0.position.z, odom1.position.z, proportion)

        (roll0, pitch0, yaw0) = euler_from_quaternion(
            [odom0.orientation.x, odom0.orientation.y, odom0.orientation.z, odom0.orientation.w])
        (roll1, pitch1, yaw1) = euler_from_quaternion(
            [odom1.orientation.x, odom1.orientation.y, odom1.orientation.z, odom1.orientation.w])
        roll = slerp(roll0, roll1, proportion)
        pitch = slerp(pitch0, pitch1, proportion)
        yaw = slerp(yaw0, yaw1, proportion)
        q = quaternion_from_euler(roll, pitch, yaw)
        rotation = [[1.0 - 2.0 * q[1] * q[1] - 2.0 * q[2] * q[2], 2.0 * q[0] * q[1] - 2.0 * q[2] * q[3],
                     2.0 * q[0] * q[2] + 2.0 * q[1] * q[3]],
                    [2.0 * q[0] * q[1] + 2.0 * q[2] * q[3], 1.0 - 2.0 * q[0] * q[0] - 2.0 * q[2] * q[2],
                     2.0 * q[1] * q[2] - 2.0 * q[0] * q[3]],
                    [2.0 * q[0] * q[2] - 2.0 * q[1] * q[3], 2.0 * q[1] * q[2] + 2.0 * q[0] * q[3],
                     1.0 - 2.0 * q[0] * q[0] - 2.0 * q[1] * q[1]]]
        translation = np.asarray([x, y, z])
        Tr_imu_to_world = np.concatenate((rotation, translation.reshape(1, -1).T), axis=1)
        Tr_imu_to_world = np.concatenate((Tr_imu_to_world, [[0., 0., 0., 1.0]]), axis=0)
        pose = (x, y, z, roll, pitch, yaw)
        return Tr_imu_to_world, pose

    for odom_data in odom_list:
        odom_timestamp = odom_data.timestamp.to_sec()
        if odom_timestamp<=timestamp:
            odom0 = odom_data.message
        if odom_timestamp>=timestamp:
            odom1 = odom_data.message

    t0 = msg_to_timestamp(odom0).to_sec()
    t1 = msg_to_timestamp(odom1).to_sec()
    if t0 == t1:
        proportion = 0
    else:
        proportion = (timestamp - t0) / (t1 - t0)
    Tr_imu_to_world, pose = getPoseFromOdom(odom0.pose.pose, odom1.pose.pose, proportion)
    return Tr_imu_to_world, pose


def compressed_imgmsg_to_cv2(cmprs_img_msg, str=False):
    import cv2
    import numpy as np
    if str:
        str_msg = cmprs_img_msg
    else:
        str_msg = cmprs_img_msg.data
    buf = np.ndarray(shape=(1, len(str_msg)),
                     dtype=np.uint8, buffer=str_msg)
    im = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
    return im


class FakeTimeStamp(object):
    def __init__(self, time):
        if hasattr(time, 'to_sec'):
            time = time.to_sec()
        self.time = time

    def to_sec(self):
        return self.time


def parse_raw_message(msg):
    message = msg.message
    message_type = message[0]
    if message_type.startswith('ros:'):
        message_type = message_type[4:]

    if message_type in message_register:
        obj_cls = message_register[message_type]
        obj = obj_cls()
        obj.deserialize(message[1])
        return True, obj
    else:
        return False, msg


def get_msg_timestamp(msg):
    if hasattr(msg.message, 'header'):
        timestamp = msg.message.header.stamp.to_sec()
    else:
        timestamp = msg.timestamp.to_sec()
    return timestamp


class BagHandler(object):
    def __init__(self, bag_path, normal_topics=None, odom_topics=None, tolerance=0.05):
        self.bag_path = bag_path
        if not normal_topics:
            self.normal_topics = []
        else:
            self.normal_topics = normal_topics
        if not odom_topics:
            self.odom_topics = []
        else:
            self.odom_topics = odom_topics
        self.tolerance = tolerance
        # self.bridge = CvBridge()
        if self.bag_path.endswith('.bag'):
            self.bag = RosBag(self.bag_path)
        else:
            self.bag = FastBag(self.bag_path)
    
    @staticmethod
    def filter_topic_exist(bag_path, topics):
        if bag_path.endswith('.bag'):
            bag = RosBag(bag_path)
        else:
            bag = FastBag(bag_path)
        filter_topic = []
        for tp in topics:
            if bag.check_topic_exists(tp):
                filter_topic.append(tp)
        return filter_topic            

    def check_calibration_topic_exist(self):
        calibration_topics = ["/perception/calibrations", "/perception/ot_calibrations"]
        for topic in calibration_topics:
            if self.bag.check_topic_exists(topic):
                return True
        return False

    def auto_add_image_topics(self):
        left_topic = self.bag.guess_left_image_topic()
        if left_topic:
            self.normal_topics.append(left_topic)
        right_topic = self.bag.guess_right_image_topic()
        if right_topic:
            self.normal_topics.append(right_topic)

    def set_normal_topics(self, topics):
        self.normal_topics = topics

    def add_normal_topics(self, topics):
        self.normal_topics.extend(topics)

    def set_odom_topics(self, odom_topics):
        self.odom_topics = odom_topics

    def msg_to_image(self, msg):
        return compressed_imgmsg_to_cv2(msg)

    def msg_generator(self, raw=False, try_convert_raw=False, set_header_timestamp=False):
        all_topics = []
        all_topics.extend(self.normal_topics)
        all_topics.extend(self.odom_topics)
        main_topic = all_topics[0]
        buffers = collections.OrderedDict([(t, []) for t in all_topics])

        if not self.normal_topics and self.odom_topics:
            for msg in self.bag.msg_generator(topics=all_topics, raw=raw):
                yield {msg.topic: [msg]}
            return

        for msg in self.bag.msg_generator(topics=all_topics, raw=raw):
            if msg.topic in all_topics:
                if raw:
                    if try_convert_raw:
                        success, obj = parse_raw_message(msg)
                        if not success:
                            msg = BagMessage(msg.topic, msg.message, FakeTimeStamp(msg.timestamp))
                        else:
                            msg = BagMessage(msg.topic, obj, FakeTimeStamp(msg.timestamp))
                    else:
                        msg = BagMessage(msg.topic, msg.message, FakeTimeStamp(msg.timestamp))
                if set_header_timestamp:
                    msg = BagMessage(msg.topic, msg.message, FakeTimeStamp(get_msg_timestamp(msg)))
                buffers[msg.topic].append(msg)
            else:
                continue
            while all(buffers.values()):
                # main_timestamp = self.msg_to_timestamp(buffers[main_topic][0].message).to_sec()
                # main_timestamp = buffers[main_topic][0].timestamp.to_sec()
                # if hasattr(buffers[main_topic][0].message, 'header'):
                #     main_timestamp = buffers[main_topic][0].message.header.stamp.to_sec()
                # else:
                #     main_timestamp = buffers[main_topic][0].timestamp.to_sec()
                main_timestamp = get_msg_timestamp(buffers[main_topic][0])
                # odom topic is special, we need both before and after
                if self.odom_topics:
                    found_odom_after_main_topic_timestamp = False
                    for topic in self.odom_topics:
                        buf = buffers[topic]
                        for data in buf:
                            # if self.msg_to_timestamp(data.message).to_sec() - main_timestamp > 2*self.tolerance:
                            if get_msg_timestamp(data) - main_timestamp >= 0:
                                found_odom_after_main_topic_timestamp = True
                                break
                        if found_odom_after_main_topic_timestamp:
                            break
                    if not found_odom_after_main_topic_timestamp:
                        break

                msg_set = {t: [] for t in all_topics}
                pop_main_topic = False
                # handle odom first
                if self.odom_topics:
                    for topic in self.odom_topics:
                        last_timestamp = main_timestamp - 0.2
                        next_timestamp = main_timestamp + 0.2
                        last_odom = None
                        next_odom = None
                        buf = buffers[topic]
                        for data in buf:
                            # msg_timestamp = self.msg_to_timestamp(data.message).to_sec()
                            msg_timestamp = get_msg_timestamp(data)
                            if 2 * self.tolerance > msg_timestamp-main_timestamp >= 0:
                                if msg_timestamp < next_timestamp:
                                    next_timestamp = msg_timestamp
                                    next_odom = data
                            if -2 * self.tolerance < msg_timestamp-main_timestamp <= 0:
                                if msg_timestamp > last_timestamp:
                                    last_timestamp = msg_timestamp
                                    last_odom = data
                        if last_odom is not None and next_odom is not None:
                            # msg_set[topic] = []
                            msg_set[topic].append(last_odom)
                            msg_set[topic].append(next_odom)
                        else:
                            pop_main_topic = True

                if self.normal_topics:
                    for topic in self.normal_topics:
                        if topic == main_topic:
                            msg_set[topic].append(buffers[topic][0])
                        else:
                            buf = buffers[topic]
                            found_topic_match_main = False
                            found_topic_after_main = False
                            for i in range(len(buf)-1, -1, -1):
                                # m = buf[i].message
                                # time_stamp = self.msg_to_timestamp(m).to_sec()
                                # time_stamp = buf[i].timestamp.to_sec()
                                time_stamp = get_msg_timestamp(buf[i])
                                time_stamp_diff = main_timestamp - time_stamp
                                if abs(time_stamp_diff) < self.tolerance:
                                    msg_set[topic].append(buf[i])
                                    found_topic_match_main = True
                                if time_stamp_diff >= self.tolerance:
                                    buf.pop(i)
                                if time_stamp_diff < 0:
                                    found_topic_after_main = True

                            if not found_topic_match_main:
                                if found_topic_after_main:
                                    pop_main_topic = True
                                else:
                                    buffers[topic] = []

                if all(msg_set.values()):
                    for topic in self.normal_topics:
                        if topic == main_topic and buffers[topic]:
                            buffers[topic].pop(0)
                    for topic in self.odom_topics:
                        for i in range(len(buffers[topic]) - 1, -1, -1):
                            # m = buffers[topic][i].message
                            # time_stamp = self.msg_to_timestamp(m).to_sec()
                            # time_stamp = buffers[topic][i].timestamp.to_sec()
                            time_stamp = get_msg_timestamp(buffers[topic][i])
                            time_stamp_diff = main_timestamp - time_stamp
                            if time_stamp_diff > 1:
                                buffers[topic].pop(i)
                    yield msg_set
                else:
                    if pop_main_topic:
                        buffers[main_topic].pop(0)


from scipy.spatial.transform import Rotation
def matrix_from_pose(pose):
    x, y, z, roll, pitch, yaw = pose
    rot = Rotation.from_euler('XYZ', [roll, pitch, yaw])
    result = np.eye(4)
    result[:3, :3] = rot.as_dcm()
    result[0, -1] = x
    result[1, -1] = y
    result[2, -1] = z
    return result


def pose_from_matrix(matrix):
    rot = Rotation.from_dcm(matrix[:3, :3])
    rads = rot.as_euler('XYZ')
    return (matrix[0, -1], matrix[1, -1], matrix[2, -1], rads[0], rads[1], rads[2])


def relative_pose(base, dst):
    m1 = matrix_from_pose(base)
    m2 = matrix_from_pose(dst)
    ret = np.matmul(np.linalg.inv(m1), m2)
    return pose_from_matrix(ret)


def homo_mat(points, mat):
    n = points.shape[0]
    points = np.concatenate((points.transpose((1, 0)), np.ones((1, n))))
    new_points = np.matmul(mat, points)
    new_points = new_points/new_points[-1]
    return new_points[:-1].transpose((1, 0))


class ImageBagHandler(object):
    def __init__(self, bag_path, front_left=False, front_right=False,
                 rear_left=False, rear_right=False,
                 side_left=False, side_right=False,
                 odom=False, other_topics=None, tolerance=0.05):
        bag_handler = BagHandler(bag_path, tolerance=tolerance)
        bag = bag_handler.bag
        image_topics = [
            (front_left, "front_left", bag.guess_left_image_topic()),
            (front_right, "front_right", bag.guess_right_image_topic()),
            (rear_left, "rear_left", bag.guess_rear_left_image_topic()),
            (rear_right, "rear_right", bag.guess_rear_right_image_topic()),
            (side_left, "side_left", bag.guess_side_left_image_topic()),
            (side_right, "side_right", bag.guess_side_right_image_topic())
        ]
        self.image_topic_dict = collections.OrderedDict()
        self.image_topic_dict_inverse = collections.OrderedDict()
        for k, n, v in image_topics:
            if k:
                if v is None:
                    raise AttributeError("did not found topic for {}!".format(n))
                bag_handler.add_normal_topics([v])
                self.image_topic_dict[n] = v
                self.image_topic_dict_inverse[v] = n

        self.odom_topic = None
        if odom:
            self.odom_topic = '/navsat/odom'
            bag_handler.set_odom_topics([self.odom_topic])

        if other_topics is None:
            other_topics = []
        self.other_topics = other_topics
        bag_handler.add_normal_topics(self.other_topics)
        self.bag_handler = bag_handler

    def handle_images(self, msg_dict, result, to_image):
        for k, v in self.image_topic_dict_inverse.items():
            img_msg = msg_dict[k][0]
            topic, message, timestamp = img_msg
            if to_image:
                # img_str = deserialize_compress_image(message[1])
                image = compressed_imgmsg_to_cv2(message.data, True)
                result[v] = BagMessage(topic, image, timestamp)
            else:
                result[v] = img_msg

    def handle_odom(self, msg_dict, result, process_odom):
        if self.odom_topic:
            if process_odom:
                odom_msgs = msg_dict[self.odom_topic]
                first_image_topic = list(self.image_topic_dict_inverse.keys())[0]
                first_image_ts = msg_dict[first_image_topic][0].timestamp.to_sec()
                imu_to_world, pose = msg_to_odom(odom_msgs, first_image_ts)
                result['imu_to_world'] = imu_to_world
                result['pose'] = pose
            else:
                result[self.odom_topic] = msg_dict[self.odom_topic]

    def msg_generator(self, process_odom=True, to_image=True):
        for msg_dict in self.bag_handler.msg_generator(raw=True, try_convert_raw=True, set_header_timestamp=True):
            result = {}
            self.handle_images(msg_dict, result, to_image)
            self.handle_odom(msg_dict, result, process_odom)
            for topic in self.other_topics:
                result[topic] = msg_dict[topic]
            yield result


if __name__ == '__main__':
    import cv2
    odom_topic = '/navsat/odom'
    # bag_path = "/home/plusai/Downloads/20210620T021649_j7-l4e-b0006_1_1710to1830.db"
    bag_path = "/home/plusai/Downloads/20210901T131509_j7-l4e-b0006_3_2700to2760.db"
    # bag_path = "/home/plusai/Downloads/20210831T100057_j7-l4e-b0006_5_2757to2817.db"
    dbw_topic = '/vehicle/dbw_reports'
    localization_topic = '/localization/status_report'
    lane_topic = '/perception/lane_path'
    # watchdog_current_topic = '/watchdog/current_state'
    bag_handler = BagHandler(bag_path, odom_topics=['/navsat/odom'], tolerance=0.05)
    # bag_handler.auto_add_image_topics()
    bag_handler.add_normal_topics([lane_topic])
    # bag_handler.add_normal_topics([dbw_topic, localization_topic])

    # from monitor import app_watchdog_state_pb2
    # from control import dbw_reports_pb2
    # from monitor.status_report_msg_pb2 import StatusReport, EnvironmentState
    from perception.lane_detection_pb2 import LaneDetection
    roll_list = []
    left_topic = bag_handler.bag.guess_left_image_topic()
    right_topic = bag_handler.bag.guess_right_image_topic()
    left_points = []
    right_points = []
    first_imu = None
    for idx, msg_dict in enumerate(bag_handler.msg_generator()):
        # left_image = bag_handler.msg_to_image(msg_dict[left_topic][0].message)
        # right_image = bag_handler.msg_to_image(msg_dict[right_topic][0].message)
        # left_image_timestamp = msg_dict[left_topic][0].timestamp.to_sec()
        # right_image_timestamp = msg_dict[right_topic][0].timestamp.to_sec()
        # imu_to_world, pose = bag_handler.msg_to_odom(msg_dict[odom_topic], left_image_timestamp)
        # x, y, z, roll, pitch, yaw = pose
        timestamp = msg_dict[lane_topic][0].timestamp.to_sec()
        imu_to_world, imu_pose = bag_handler.msg_to_odom(msg_dict[odom_topic], timestamp)
        lane_detection = LaneDetection()
        lane_detection.ParseFromString(msg_dict[lane_topic][0].message.data)
        ego_lane_id = lane_detection.ego_lane_id

        if first_imu is None:
            first_imu = imu_pose
        # imu_pose = relative_pose(first_imu, imu_pose)

        for lane in lane_detection.lane:
            if lane.lane_id != lane_detection.ego_lane_id:
                continue
            if lane.image_left_boundary is None or len(lane.image_left_boundary) == 0 or \
                    lane.image_right_boundary is None or len(lane.image_right_boundary) == 0:
                # if lane.image_center_curve is None or len(lane.image_center_curve) == 0 :
                pass
            else:
                is_lane_result_good = True
                # print(len(lane.left_boundary.curve.segment[0].line_segment.point))
                for idx, point in enumerate(lane.left_boundary.curve.segment[0].line_segment.point):
                    if idx == 60:
                        left_points.append((point.x, point.y, point.z))

                for idx, point in enumerate(lane.right_boundary.curve.segment[0].line_segment.point):
                    if idx == 60:
                        right_points.append((point.x, point.y, point.z))


    first_matrix = matrix_from_pose(first_imu)
    left_points.extend(right_points)
    left_points = np.array(left_points)
    left_points = homo_mat(left_points, np.linalg.inv(first_matrix))

    from drive_python.utils.pcl_utils import writePcd
    writePcd(left_points, "/home/plusai/temp/heading/cache10/20210901T131509_j7-l4e-b0006_3_2700to2760.db/lane2.pcd")
    # writePcd(left_points, "/home/plusai/temp/heading/cache10/20210831T100057_j7-l4e-b0006_5_2757to2817.db/lane2.pcd")



    # import matplotlib.pyplot as plt
    # plt.plot(roll_list)
    # plt.show()
        # dbw_reports = dbw_reports_pb2.DbwReports()
        # dbw_reports.ParseFromString(msg_dict[dbw_topic][0].message.data)
        # localization_status_report = StatusReport()
        # localization_status_report.ParseFromString(msg_dict[localization_topic][0].message.data)
        # is_superpilot = dbw_reports.superpilot_enabled
        # print("superpilot status: {}".format(is_superpilot))
        #
        # env_values = {env.state_type: env.value for env in localization_status_report.environment_states}
        # road_type_key = EnvironmentState.SUPPORTED_ROAD_TYPE if 'SUPPORTED_ROAD_TYPE' in dir(
        #     EnvironmentState) else 501
        # on_supported_road = env_values[road_type_key] > 0 if road_type_key in env_values else False
        # print("on_supported_road: {}".format(on_supported_road))

        # cv2.imshow("left", left_image)
        # cv2.imshow("right", right_image)
        # cv2.waitKey()



