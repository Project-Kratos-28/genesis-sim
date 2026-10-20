import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
 
from . import config
from .pointcloud import depth_to_pointcloud

def _to_xyz(v):
    if hasattr(v, "detach"):
        v = v.detach().cpu().numpy()
    v = np.asarray(v).reshape(-1)
    if len(v) != 3:
        raise ValueError(f"Expected 3 IMU values, got {v}")
    return [float(x) for x in v]


class BridgePublishers:
    def __init__(self, node, camera_model):
        self.node = node
        self.cam = camera_model
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(node)
 
        mk = node.create_publisher
        self.left_img = mk(Image, config.TOPIC_LEFT_IMAGE, 10)
        self.right_img = mk(Image, config.TOPIC_RIGHT_IMAGE, 10)
        self.left_info = mk(CameraInfo, config.TOPIC_LEFT_INFO, 10)
        self.right_info = mk(CameraInfo, config.TOPIC_RIGHT_INFO, 10)
        self.imu_raw = mk(Imu, config.TOPIC_IMU_RAW, 10)
        self.clock = mk(ClockMsg, config.TOPIC_CLOCK, 10)
        # self.odom = mk(Odometry, config.TOPIC_ODOM, 10)
        self.left_pc = mk(PointCloud2, config.TOPIC_LEFT_PC, 10)
        self.right_pc = mk(PointCloud2, config.TOPIC_RIGHT_PC, 10)
 
    # --- state / time -------------------------------------------------------
    def publish_clock(self):
        msg = ClockMsg()
        msg.clock = self.node.stamp()
        self.clock.publish(msg)
 
    def publish_base_tf(self, pos, quat):
        t = TransformStamped()
        t.header.stamp = self.node.stamp()
        t.header.frame_id = config.MAP_FRAME
        t.child_frame_id = config.BASE_FOOTPRINT_FRAME
        self._fill_pose(t.transform.translation, t.transform.rotation, pos, quat)
        self.tf_broadcaster.sendTransform(t)
 
    def publish_odom(self, pos, quat):
        msg = Odometry()
        msg.header.stamp = self.node.stamp()
        msg.header.frame_id = config.MAP_FRAME
        msg.child_frame_id = config.ODOM_CHILD_FRAME
        p = msg.pose.pose
        self._fill_pose(p.position, p.orientation, pos, quat)
        # self.odom.publish(msg)
 
    @staticmethod
    def _fill_pose(trans, rot, pos, quat):
        trans.x, trans.y, trans.z = float(pos[0]), float(pos[1]), float(pos[2])
        # Genesis quats are [w, x, y, z]; ROS wants x, y, z, w.
        rot.x, rot.y, rot.z, rot.w = (float(quat[1]), float(quat[2]),
                                      float(quat[3]), float(quat[0]))
 
    # --- IMU ----------------------------------------------------------------
    def publish_imu(self, imu_data, stamp=None, orientation=None):
        if stamp is None:
            stamp = self.node.stamp()

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = config.IMU_FRAME
 
        lin_acc = getattr(imu_data, "lin_acc", None)
        if lin_acc is None:
            lin_acc = imu_data.linear_acceleration
        ang_vel = getattr(imu_data, "ang_vel", None)
        if ang_vel is None:
            ang_vel = imu_data.angular_velocity
 
        acc, gyro = _to_xyz(lin_acc), _to_xyz(ang_vel)


        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro

        if orientation is None:
            orientation = (0.0, 0.0, 0.0, 1.0)

        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]

        msg.orientation_covariance = [
            1e-6, 0.0, 0.0,
            0.0, 1e-6, 0.0,
            0.0, 0.0, 1e-6
        ]

        self.imu_raw.publish(msg)
 
    # --- cameras ------------------------------------------------------------
    def _publish_image(self, pub, rgb, frame_id, stamp):
        msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(rgb), encoding="passthrough")
        msg.encoding = "rgb8"
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        pub.publish(msg)
 
    def _publish_info(self, pub, frame_id, P, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width, info.height = self.cam.width, self.cam.height
        info.k = self.cam.K
        info.p = P
        pub.publish(info)
 
    def _publish_cloud(self, pub, pts, frame_id, stamp):
        pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=frame_id), pts))
 
    def publish_stereo(self, rgb_l, depth_l, rgb_r, depth_r, stamp=None):
        if stamp is None:
            stamp = self.node.stamp()
        L, R = config.LEFT_CAM_FRAME, config.RIGHT_CAM_FRAME
 
        self._publish_image(self.left_img, rgb_l, L, stamp)
        self._publish_image(self.right_img, rgb_r, R, stamp)
        self._publish_info(self.left_info, L, self.cam.P_left, stamp)
        self._publish_info(self.right_info, R, self.cam.P_right, stamp)
        self._publish_cloud(self.left_pc, depth_to_pointcloud(depth_l, self.cam), L, stamp)
        self._publish_cloud(self.right_pc, depth_to_pointcloud(depth_r, self.cam), R, stamp)