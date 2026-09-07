#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import genesis as gs
import os
import math
import random
import tempfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, CameraInfo
from rosgraph_msgs.msg import Clock as ClockMsg
from builtin_interfaces.msg import Time
import numpy as np
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

PACKAGE_SHARE = get_package_share_directory("athena_description")
URDF_PATH = os.path.join(PACKAGE_SHARE, "urdf", "athena_rover-6.urdf")
GROUND_TEXTURE_PATH = os.path.join(PACKAGE_SHARE, "ground_noise.png")

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

def create_genesis_urdf():
    with open(URDF_PATH, "r") as urdf_file:
        urdf = urdf_file.read()

    urdf = urdf.replace(
        "package://athena_description",
        PACKAGE_SHARE,
    )
    file_descriptor, resolved_urdf_path = tempfile.mkstemp(
        suffix=".urdf",
        prefix="athena_rover_",
    )
    with os.fdopen(file_descriptor, "w") as resolved_urdf_file:
        resolved_urdf_file.write(urdf)
    return resolved_urdf_path

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
    def __init__(self, width, height, fov_deg):
        super().__init__("genesis_zed_bridge")
        self.left_cam_pub_ = self.create_publisher(Image, "/zed2i/left/image_rect_color", 10)
        self.right_cam_pub_ = self.create_publisher(Image, "/zed2i/right/image_rect_color", 10)
        self.left_cam_info_pub_ = self.create_publisher(CameraInfo, "/zed2i/left/camera_info", 10)
        self.right_cam_info_pub_ = self.create_publisher(CameraInfo, "/zed2i/right/camera_info", 10)
        self.imu_pub_ = self.create_publisher(Imu, "/zed2i/imu/data", 10)
        self.clock_pub_ = self.create_publisher(ClockMsg, "/clock", 10)

        self.cmd_vel_sub_ = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.v = 0.0
        self.wz = 0.0


        self.bridge = CvBridge()
        self.width = width
        self.height = height
        fov = np.deg2rad(fov_deg)
        fx = fy = (width/2.0) / np.tan(fov/2.0)

        cx, cy = width/2.0, height/2.0

        self.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.P_left  = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.P_right = [fx, 0.0, cx, -fx*BASELINE, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.sim_time = 0.0



    def stamp(self):
        sec = int(self.sim_time)
        nsec = int((self.sim_time - sec) * 1e9)
        return Time(sec=sec, nanosec=nsec)

    def on_cmd_vel(self, msg):
        self.v = msg.linear.x
        self.wz = msg.angular.z


    def publish_clock(self):
        msg = ClockMsg()
        msg.clock = self.stamp()
        self.clock_pub_.publish(msg)

    def publish_camera_info(self, pub, frame_id, P):
        info = CameraInfo()
        info.header.stamp = self.stamp()
        info.header.frame_id = frame_id
        info.width, info.height = self.width, self.height
        info.k = self.K
        info.p = P
        pub.publish(info)

    def publish_image(self, pub, rgb, frame_id):
        msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(rgb), encoding="passthrough")
        msg.encoding = "rgb8"
        msg.header.stamp = self.stamp()
        msg.header.frame_id = frame_id
        pub.publish(msg)
 
    def publish_imu(self, imu_data, frame_id):
        msg = Imu()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = frame_id
        lin_acc = getattr(imu_data, "lin_acc", None)
        if lin_acc is None:
            lin_acc = imu_data.linear_acceleration
        ang_vel = getattr(imu_data, "ang_vel", None)
        if ang_vel is None:
            ang_vel = imu_data.angular_velocity
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = [float(v) for v in lin_acc]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = [float(v) for v in ang_vel]
        msg.orientation_covariance[0] = -1.0 
        self.imu_pub_.publish(msg)


    

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
            diffuse_texture=gs.textures.ImageTexture(image_path=GROUND_TEXTURE_PATH)
        )
    )

    for i in range(15):
        x = random.uniform(1.0, 6.0)
        y = random.uniform(3.0, 10.0)
        scene.add_entity(
            gs.morphs.Box(pos=(x, y, 0.15), size=(0.3, 0.3, 0.3), fixed=True),
            surface=gs.surfaces.Rough(color=(random.random(), random.random(), random.random(), 1.0)),
        )
    resolved_urdf_path = create_genesis_urdf()
    try:
        rover = scene.add_entity(
            gs.morphs.URDF(
                file=resolved_urdf_path,
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
    finally:
        os.unlink(resolved_urdf_path)

    steer_dofs = [rover.get_joint(name).dofs_idx_local[0] for name in STEER_JOINTS]
    wheel_dofs = [rover.get_joint(name).dofs_idx_local[0] for name in WHEEL_JOINTS]

    rover.set_dofs_kp(kp=[800.0] * 4, dofs_idx_local=steer_dofs)
    rover.set_dofs_kv(kv=[40.0] * 4, dofs_idx_local=steer_dofs)
    rover.control_dofs_position(position=[0.0] * 4, dofs_idx_local=steer_dofs)

    rover.set_dofs_kp(kp=[0.0] * 4, dofs_idx_local=wheel_dofs)
    rover.set_dofs_kv(kv=[30.0] * 4, dofs_idx_local=wheel_dofs)

    node = GenesisZedBridge(WIDTH, HEIGHT, FOV_DEG)

    try :
        while rclpy.ok():
            steer_angles, wheel_ang_vels = compute_wheel_states(node.v, node.wz)
            rover.control_dofs_position(position=steer_angles, dofs_idx_local=steer_dofs)
            rover.control_dofs_velocity(velocity=wheel_ang_vels, dofs_idx_local=wheel_dofs)

            scene.step()
            left_cam.move_to_attach()
            right_cam.move_to_attach()

            node.sim_time+=SIM_DT

            node.publish_clock()

            rgb_l, _, _, _ = left_cam.render(rgb=True)
            rgb_r, _, _, _ = right_cam.render(rgb=True)

            node.publish_image(node.left_cam_pub_, rgb_l, "zed2i_left_camera_frame_optical")
            node.publish_image(node.right_cam_pub_, rgb_r, "zed2i_right_camera_frame_optical")
            node.publish_camera_info(node.left_cam_info_pub_, "zed2i_left_camera_frame_optical", node.P_left)
            node.publish_camera_info(node.right_cam_info_pub_, "zed2i_right_camera_frame_optical", node.P_right)

            imu_data = imu.read() if hasattr(imu, "read") else imu.get_data()
            node.publish_imu(imu_data, "zed2i_camera_center")

            rclpy.spin_once(node, timeout_sec=0)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()