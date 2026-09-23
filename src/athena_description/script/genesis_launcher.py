#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy
from rclpy.node import Node
import genesis as gs
import os
import math
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, CameraInfo, PointCloud2
from std_msgs.msg  import Header, String
from sensor_msgs_py import point_cloud2
from rosgraph_msgs.msg import Clock as ClockMsg
from builtin_interfaces.msg import Time
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry

from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from steering import make_steering_mode, STEERING_MODES
from steering.wheel_config import STEER_JOINTS, WHEEL_JOINTS

URDF_PATH = os.path.expanduser("~/Desktop/kratos/src/athena_description/urdf/athena_rover-6.urdf")


WIDTH, HEIGHT, FOV_DEG = 1280, 720, 110.0
BASELINE = 0.12

SIM_DT=0.01


WHEEL_BOTTOM_TO_FOOTPRINT = -0.2145
SPAWN_CLEARANCE = 0.03
SPAWN_Z = -WHEEL_BOTTOM_TO_FOOTPRINT + SPAWN_CLEARANCE

WHEEL_RADIUS = 0.115

WHEEL_XY = {
    "front_left":  (-0.39291,  0.39181),
    "front_right": (-0.39291, -0.39181),
    "rear_left":   ( 0.34458,  0.37898),
    "rear_right":  ( 0.34458, -0.37898),
}

WHEEL_ORDER = ["front_left", "front_right", "rear_left", "rear_right"]
 
STEER_JOINTS = ["steer_front_left", "steer_front_right", "steer_rear_left", "steer_rear_right"]
WHEEL_JOINTS = ["wheel_front_left_spin", "wheel_front_right_spin", "wheel_rear_left_spin", "wheel_rear_right_spin"]
 
MAX_STEER_ANGLE = 1.57

CAMERA_DECIMATION = 5

DEFAULT_STEERING_MODE = "ackermann"


def depth_to_pointcloud(depth, fx, fy, cx, cy, stride=4):
    h, w = depth.shape
    us, vs = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
    z = depth[vs, us]
    valid = (z > 0.05) & (z < 30.0) & np.isfinite(z)
    x = -((us - cx) * z / fx)
    y = ((vs - cy) * z / fy)
    return np.stack([-x[valid], -y[valid], z[valid]], axis=-1).astype(np.float32)

def publish_cloud(pub, pts, stamp, frame_id):
    header = Header(stamp=stamp, frame_id=frame_id)
    msg = point_cloud2.create_cloud_xyz32(header, pts)
    pub.publish(msg)


def compute_wheel_states(v, wz):
    steer_angles, wheel_ang_vels = [], []
    for name in WHEEL_ORDER:
        x, y = WHEEL_XY[name]
        vx = v - wz * y
        vy = wz * x
        angle = math.atan2(vy, vx)
        speed = math.hypot(vx, vy)
        if abs(angle) > math.pi / 2:
            angle = math.atan2(-vy, -vx)
            speed = -speed
        angle = max(-MAX_STEER_ANGLE, min(MAX_STEER_ANGLE, angle))
        steer_angles.append(angle)
        wheel_ang_vels.append(speed / WHEEL_RADIUS)
    return steer_angles, wheel_ang_vels



class GenesisZedBridge(Node):
    def __init__(self, width, height, fov_deg, rover):
        super().__init__("genesis_zed_bridge")

        self.rover = rover
        self.left_cam_pub_ = self.create_publisher(Image, "/zed2i/left/image_rect_color", 10)
        self.right_cam_pub_ = self.create_publisher(Image, "/zed2i/right/image_rect_color", 10)
        self.left_cam_info_pub_ = self.create_publisher(CameraInfo, "/zed2i/left/camera_info", 10)
        self.right_cam_info_pub_ = self.create_publisher(CameraInfo, "/zed2i/right/camera_info", 10)
        self.imu_pub_ = self.create_publisher(Imu, "/zed2i/imu/data", 10)
        self.clock_pub_ = self.create_publisher(ClockMsg, "/clock", 10)
        self.temp_odom_pub_ = self.create_publisher(Odometry, '/odom', 10)

        self.left_pc_pub_ = self.create_publisher(PointCloud2, '/point_cloud/left', 10)
        self.right_pc_pub_ = self.create_publisher(PointCloud2, '/point_cloud/right', 10)


        self.cmd_vel_sub_ = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.v = 0.0
        self.wz = 0.0

        self.target_v = 0.0
        self.target_wz = 0.0


        self.declare_parameter("steering_mode", DEFAULT_STEERING_MODE)
        requested_mode = self.get_parameter("steering_mode").value

        try:
            self.steering = make_steering_mode(requested_mode)
        except ValueError as e :
            self.get_logger().warn(f"{e}; falling back to '{DEFAULT_STEERING_MODE}'")
            self.steering = make_steering_mode(DEFAULT_STEERING_MODE)
        self.get_logger().info(f"Steering mode: {self.steering.name}")

        self.steering_mode_sub_ = self.create_subscription(
            String, "/steering_mode", self.on_steering_mode, 10
        )


        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.width = width
        self.height = height
        fov = np.deg2rad(fov_deg)
        self.fx = self.fy = (width/2.0) / np.tan(fov/2.0)

        self.cx, self.cy = width/2.0, height/2.0

        self.K = [
            self.fx, 0.0, self.cx, 
            0.0, self.fy, self.cy, 
            0.0, 0.0, 1.0
        ]
        self.P_left  = [
            self.fx, 0.0, self.cx, 0.0, 
            0.0, self.fy, self.cy, 0.0, 
            0.0, 0.0, 1.0, 0.0
        ]
        self.P_right = [
            self.fx, 0.0, self.cx, -self.fx*BASELINE, 
            0.0, self.fy, self.cy, 0.0, 
            0.0, 0.0, 1.0, 0.0
        ]

        self.sim_time = 0.0


    def on_steering_mode(self, msg):
        requested = msg.data.strip().lower()
        if requested == self.steering.name:
            return
        try:
            self.steering = make_steering_mode(requested)
            self.get_logger().info(f"Switched steering mode -> {self.steering.name}")
        except ValueError as e:
            self.get_logger().warn(str(e))



    def stamp(self):
        sec = int(self.sim_time)
        nsec = int((self.sim_time - sec) * 1e9)
        return Time(sec=sec, nanosec=nsec)

    def on_cmd_vel(self, msg):
        self.target_v = msg.linear.x
        self.target_wz = msg.angular.z


    def publish_clock(self):
        msg = ClockMsg()
        msg.clock = self.stamp()
        self.clock_pub_.publish(msg)
        pos = self.rover.get_pos()
        quat = self.rover.get_quat()
        self.publish_base_tf(pos, quat)

    def publish_camera_info(self, pub, frame_id, P):
        info = CameraInfo()
        info.header.stamp = self.stamp()
        info.header.frame_id = frame_id
        info.width, info.height = self.width, self.height
        info.k = self.K
        info.p = P
        pub.publish(info)

    def publish_image(self, pub, rgb, frame_id, stamp):
        msg = self.bridge.cv2_to_imgmsg(
        np.ascontiguousarray(rgb),
            encoding="passthrough"
        )

        msg.encoding = "rgb8"
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id

        pub.publish(msg)

    def tensor_to_xyz(self, v):
        if hasattr(v, "detach"):
            v = v.detach().cpu().numpy()

        v = np.asarray(v).reshape(-1)

        if len(v) != 3:
            raise ValueError(f"Expected 3 IMU values, got {v}")

        return [float(x) for x in v]
 
    def publish_imu(self, imu_data, frame_id):
        msg = Imu()

        msg.header.stamp = self.stamp()
        msg.header.frame_id = frame_id

        lin_acc = getattr(imu_data, "lin_acc", None)
        ang_vel = getattr(imu_data, "ang_vel", None)

        if lin_acc is None:
            lin_acc = imu_data.linear_acceleration

        if ang_vel is None:
            ang_vel = imu_data.angular_velocity

        acc = self.tensor_to_xyz(lin_acc)
        gyro = self.tensor_to_xyz(ang_vel)

        msg.linear_acceleration.x = acc[0]
        msg.linear_acceleration.y = acc[1]
        msg.linear_acceleration.z = acc[2]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.orientation_covariance[0] = -1.0

        self.imu_pub_.publish(msg)

        if int(self.sim_time * 10) % 10 == 0:
            print(
                f"IMU t={self.sim_time:.2f} "
                f"acc=[{acc[0]:+.3f}, {acc[1]:+.3f}, {acc[2]:+.3f}] "
                f"gyro=[{gyro[0]:+.3f}, {gyro[1]:+.3f}, {gyro[2]:+.3f}]"
            )

    def publish_base_tf(self, pos, quat):
        t = TransformStamped()
        t.header.stamp = self.stamp()
        t.header.frame_id = "map"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.x = float(quat[1])
        t.transform.rotation.y = float(quat[2])
        t.transform.rotation.z = float(quat[3])
        t.transform.rotation.w = float(quat[0])
        self.tf_broadcaster_.sendTransform(t)


    def publish_odom_g(self, pos, quat) :

        msg = Odometry()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'

        
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])

        msg.pose.pose.orientation.x = float(quat[1])
        msg.pose.pose.orientation.y = float(quat[2])
        msg.pose.pose.orientation.z = float(quat[3])
        msg.pose.pose.orientation.w = float(quat[0])

        self.temp_odom_pub_.publish(msg)


    

def main() :
    rclpy.init()
    gs.init(backend=gs.gpu)
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.01),
        show_viewer=True
    )
    scene.add_entity(
        gs.morphs.Plane(),
        surface=gs.surfaces.Rough(
            diffuse_texture=gs.textures.ImageTexture(image_path=os.path.expanduser("~/Desktop/kratos/src/athena_description/img/ground_noise.png"))
        )
    )


    points = [
        (2.98, -6.41), (-5.12, 3.07), (7.45, 1.22), (-1.88, -3.44),
        (4.60, 5.33), (-6.99, -0.85), (0.77, 7.88), (3.34, -2.11),
        (-4.02, -6.15), (6.80, -3.29), (-2.55, 4.71), (1.09, -7.62),
        (5.94, 3.98), (-7.31, 2.04), (-0.42, 3.05), (2.66, 6.55),
        (-3.79, -4.88), (7.02, -4.55), (-1.15, -7.99), (4.44, -6.02),
    ]

    for x, y in points: 
        scene.add_entity(
            gs.morphs.Box(pos=(x, y, 0.5), size=(0.3, 0.3, 1.0), fixed=True),
            surface=gs.surfaces.Rough(color=(random.random(), random.random(), random.random(), 1.0)),
        )
    rover = scene.add_entity(
        gs.morphs.URDF(
            file=URDF_PATH,
            fixed=False,
            pos=(0.0, 0.0, SPAWN_Z),
            merge_fixed_links=True,
            links_to_keep=[
                "base_footprint",
                "zed2i_camera_center",
                "zed2i_left_camera_frame_optical",
                "zed2i_right_camera_frame_optical"
            ]
        )
    )
    left_cam = scene.add_camera(res=(1280, 720), fov=110)
            
    right_cam = scene.add_camera(res=(1280, 720), fov=110)

    FLIP_180_Z = np.array([
        [-1,  0, 0, 0],
        [ 0, -1, 0, 0],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1],
    ], dtype=np.float64)
    

    left_cam.attach(rover.get_link("zed2i_left_camera_frame_optical"), offset_T=np.eye(4))
    right_cam.attach(rover.get_link("zed2i_right_camera_frame_optical"), offset_T=np.eye(4))

    imu_link = rover.get_link("zed2i_camera_center")

    imu = scene.add_sensor(
        gs.sensors.IMU(
            entity_idx=rover.idx,
            link_idx_local=imu_link.idx_local,
        )
    )

    scene.build()

    steer_dofs = [rover.get_joint(name).dofs_idx_local[0] for name in STEER_JOINTS]
    wheel_dofs = [rover.get_joint(name).dofs_idx_local[0] for name in WHEEL_JOINTS]

    rover.set_dofs_kp(kp=[800.0] * 4, dofs_idx_local=steer_dofs)
    rover.set_dofs_kv(kv=[40.0] * 4, dofs_idx_local=steer_dofs)
    rover.control_dofs_position(position=[0.0] * 4, dofs_idx_local=steer_dofs)

    rover.set_dofs_kp(kp=[0.0] * 4, dofs_idx_local=wheel_dofs)
    rover.set_dofs_kv(kv=[30.0] * 4, dofs_idx_local=wheel_dofs)

    node = GenesisZedBridge(WIDTH, HEIGHT, FOV_DEG, rover)

    step_count = 0

    try :
        while rclpy.ok():
            node.v = node.target_v
            node.wz = node.target_wz
            steer_angles, wheel_ang_vels = node.steering.compute(node.v, node.wz)
            rover.control_dofs_position(position=steer_angles, dofs_idx_local=steer_dofs)
            rover.control_dofs_velocity(velocity=wheel_ang_vels, dofs_idx_local=wheel_dofs)

            scene.step()

            pos = rover.get_pos()
            quat = rover.get_quat()

            node.publish_odom_g(pos, quat)
            left_cam.move_to_attach()
            right_cam.move_to_attach()

            node.sim_time+=SIM_DT

            node.publish_clock()

            imu_data = imu.read() if hasattr(imu, "read") else imu.get_data()
            node.publish_imu(imu_data, "zed2i_camera_center")


            if step_count % CAMERA_DECIMATION == 0:
                left_cam.move_to_attach()
                right_cam.move_to_attach()

                stamp = node.stamp()

                print(
                    f"STEREO FRAME "
                    f"stamp={stamp.sec}.{stamp.nanosec:09d}"
                )
 
                rgb_l, depth_l, _, _ = left_cam.render(rgb=True, depth=True)
                rgb_r, depth_r, _, _ = right_cam.render(rgb=True, depth=True)

                pts_l = depth_to_pointcloud(depth_l, node.fx, node.fy, node.cx, node.cy, stride=4)
                pts_r = depth_to_pointcloud(depth_r, node.fx, node.fy, node.cx, node.cy, stride=4)

 
                node.publish_image(node.left_cam_pub_, rgb_l, "zed2i_left_camera_frame_optical", stamp)
                node.publish_image(node.right_cam_pub_, rgb_r, "zed2i_right_camera_frame_optical", stamp)
                node.publish_camera_info(node.left_cam_info_pub_, "zed2i_left_camera_frame_optical", node.P_left)
                node.publish_camera_info(node.right_cam_info_pub_, "zed2i_right_camera_frame_optical", node.P_right)

                stamp = node.stamp()
                publish_cloud(node.left_pc_pub_, pts_l, stamp, "zed2i_left_camera_frame_optical")
                publish_cloud(node.right_pc_pub_, pts_r, stamp, "zed2i_right_camera_frame_optical")
            step_count+=1
            rclpy.spin_once(node, timeout_sec=0)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()