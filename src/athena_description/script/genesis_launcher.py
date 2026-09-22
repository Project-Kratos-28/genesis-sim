#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import genesis as gs
import os
import math
import random
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, Imu, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import numpy as np
import torch
from cv_bridge import CvBridge
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ament_index_python.packages import get_package_share_directory

URDF_PATH = os.path.expanduser("~/genesis-sim/src/athena_description/urdf/athena_rover-6.urdf")

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
LIDAR_DECIMATION = 1
LIDAR_RAYS = 524288
LIDAR_MAX_RANGE = 100.0
LIDAR_MIN_RANGE = 0.1

class LivoxRayPattern(gs.sensors.RaycastPattern):
    def __init__(self, directions):
        self._return_shape = (len(directions),)
        self._ray_dirs = torch.as_tensor(
            directions,
            dtype=torch.float32,
            device=gs.device,
        )
        self._ray_starts = torch.zeros_like(self._ray_dirs)

    def _get_return_shape(self):
        return self._return_shape

    def compute_ray_dirs(self):
        return self._ray_dirs

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
        self.lidar_pub_ = self.create_publisher(PointCloud2, "/livox/lidar", 10)
        self.livox_imu_pub_ = self.create_publisher(Imu, "/livox/imu", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.cmd_vel_sub_ = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.v = 0.0
        self.wz = 0.0


        self.bridge = CvBridge()
        self.width = width
        self.height = height
        fov = np.deg2rad(fov_deg)
        fx = fy = (width/2.0) / np.tan(fov/2.0)

        cx, cy = width/2.0, height/2.0

        self.K = [
            fx, 0.0, cx, 
            0.0, fy, cy, 
            0.0, 0.0, 1.0
        ]
        self.P_left  = [
            fx, 0.0, cx, 0.0, 
            0.0, fy, cy, 0.0, 
            0.0, 0.0, 1.0, 0.0
        ]
        self.P_right = [
            fx, 0.0, cx, -fx*BASELINE, 
            0.0, fy, cy, 0.0, 
            0.0, 0.0, 1.0, 0.0
        ]

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

    def publish_map_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "world"
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def publish_base_transform(self, base_link):
        position = self._numpy(base_link.get_pos()).reshape(3)
        w, x, y, z = self._numpy(base_link.get_quat()).reshape(4)

        transform = TransformStamped()
        transform.header.stamp = self.stamp()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = float(x)
        transform.transform.rotation.y = float(y)
        transform.transform.rotation.z = float(z)
        transform.transform.rotation.w = float(w)
        self.tf_broadcaster.sendTransform(transform)
        
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
 
    def publish_imu(self, imu_data, frame_id, publisher=None):
        if publisher is None:
            publisher = self.imu_pub_

        msg = Imu()
        msg.header.stamp = self.stamp()
        msg.header.frame_id = frame_id

        lin_acc = (
            imu_data.lin_acc
            if hasattr(imu_data, "lin_acc")
            else imu_data.linear_acceleration
        )
        ang_vel = (
            imu_data.ang_vel
            if hasattr(imu_data, "ang_vel")
            else imu_data.angular_velocity
        )

        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = [
            float(v) for v in lin_acc
        ]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = [
            float(v) for v in ang_vel
        ]
        msg.orientation_covariance[0] = -1.0
        publisher.publish(msg)
        
        
    @staticmethod
    def _numpy(value):
        if hasattr(value, "detach"):
            value = value.detach().cpu().numpy()
        return np.asarray(value, dtype=np.float32)

    @staticmethod
    def _rotation_from_quaternion(quat):
        # Genesis quaternions are expected in w, x, y, z order.
        w, x, y, z = GenesisZedBridge._numpy(quat).reshape(4)

        return np.array([
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ], dtype=np.float32)

    def publish_lidar(self, lidar_sensor):
        ray_result = lidar_sensor.read()
        distances = ray_result.distances if hasattr(ray_result, "distances") else ray_result
        distances = self._numpy(ray_result.distances).reshape(-1)
        points = self._numpy(ray_result.points).reshape(-1, 3)
        
        valid = (
            np.isfinite(distances)
            & (distances >= LIDAR_MIN_RANGE)
            & (distances < LIDAR_MAX_RANGE)
        )
        self.publish_point_cloud(points[valid], "livox_mid360_link")

    def publish_point_cloud(self, points, frame_id):
        header = Header()
        header.stamp = self.stamp()
        header.frame_id = frame_id

        cloud = point_cloud2.create_cloud_xyz32(
            header,
            [tuple(map(float, point)) for point in points],
        )
        self.lidar_pub_.publish(cloud)


def main() :
    rclpy.init()
    gs.init(backend=gs.gpu)
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.01),
        show_viewer=True
    )
    
    environment_entities = []

    scene.add_entity(
        gs.morphs.Plane(),
        surface=gs.surfaces.Rough(
            diffuse_texture=gs.textures.ImageTexture(
                image_path=os.path.expanduser(
                    "~/genesis-sim/src/athena_description/img/ground_noise.png"
                )
            )
        )
    )

    for i in range(15):
        y = random.uniform(-3.0, 3.0)
        x = random.uniform(3.0, 8.0)
        environment_entities.append(
            scene.add_entity(
                gs.morphs.Box(
                    pos=(x, y, 0.15),
                    size=(0.3, 0.3, 0.3),
                    fixed=True,
                ),
                surface=gs.surfaces.Rough(
                    color=(random.random(), random.random(), random.random(), 1.0)
                ),
            )
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
                "zed2i_right_camera_frame_optical",
                "livox_mid360_link",
            ]
        )
    )
    
    left_cam = scene.add_camera(res=(1280, 720), fov=110)        
    right_cam = scene.add_camera(res=(1280, 720), fov=110)
    

    left_cam.attach(rover.get_link("zed2i_left_camera_frame_optical"), offset_T=np.eye(4))
    right_cam.attach(rover.get_link("zed2i_right_camera_frame_optical"), offset_T=np.eye(4))

    imu_link = rover.get_link("zed2i_camera_center")
    livox_link = rover.get_link("livox_mid360_link")
    base_link = rover.get_link("base_footprint")
    
    livox = scene.add_sensor(
        gs.sensors.Raycaster(
            entity_idx=rover.idx,
            link_idx_local=livox_link.idx_local,
            pattern=gs.sensors.SphericalPattern(
                fov=(360.0, 59.0),
                n_points=(1024, 128),
            ),
            pos_offset=(0.0, 0.0, 0.19),
            max_range=LIDAR_MAX_RANGE,
            no_hit_value=LIDAR_MAX_RANGE,
            return_world_frame=False,
            return_points=True,
        )
    )

    livox_imu = scene.add_sensor(
        gs.sensors.IMU(
            entity_idx=rover.idx,
            link_idx_local=livox_link.idx_local,
        )
    )
    imu = scene.add_sensor(
        gs.sensors.IMU(
            entity_idx=rover.idx,
            link_idx_local=imu_link.idx_local,
        )
    )

    scene.build()

    print(livox_link.get_pos())

    steer_dofs = [rover.get_joint(name).dofs_idx_local[0] for name in STEER_JOINTS]
    wheel_dofs = [rover.get_joint(name).dofs_idx_local[0] for name in WHEEL_JOINTS]

    rover.set_dofs_kp(kp=[800.0] * 4, dofs_idx_local=steer_dofs)
    rover.set_dofs_kv(kv=[40.0] * 4, dofs_idx_local=steer_dofs)
    rover.control_dofs_position(position=[0.0] * 4, dofs_idx_local=steer_dofs)

    rover.set_dofs_kp(kp=[0.0] * 4, dofs_idx_local=wheel_dofs)
    rover.set_dofs_kv(kv=[30.0] * 4, dofs_idx_local=wheel_dofs)

    node = GenesisZedBridge(WIDTH, HEIGHT, FOV_DEG)
    node.publish_map_transform()

    step_count = 0

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
            node.publish_base_transform(base_link)

            imu_data = imu.read() if hasattr(imu, "read") else imu.get_data()
            node.publish_imu(imu_data, "zed2i_camera_center", node.imu_pub_)
            livox_imu_data = (
                livox_imu.read()
                if hasattr(livox_imu, "read")
                else livox_imu.get_data()
            )
            node.publish_imu(
                livox_imu_data,
                "livox_mid360_link",
                node.livox_imu_pub_,
            )

            if step_count % LIDAR_DECIMATION == 0:
                node.publish_lidar(
                livox
            )        
                
            if step_count % CAMERA_DECIMATION == 0:
                left_cam.move_to_attach()
                right_cam.move_to_attach()
 
                rgb_l, _, _, _ = left_cam.render(rgb=True)
                rgb_r, _, _, _ = right_cam.render(rgb=True)
 
                node.publish_image(node.left_cam_pub_, rgb_l, "zed2i_left_camera_frame_optical")
                node.publish_image(node.right_cam_pub_, rgb_r, "zed2i_right_camera_frame_optical")
                node.publish_camera_info(node.left_cam_info_pub_, "zed2i_left_camera_frame_optical", node.P_left)
                node.publish_camera_info(node.right_cam_info_pub_, "zed2i_right_camera_frame_optical", node.P_right)
            step_count+=1
            rclpy.spin_once(node, timeout_sec=0)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()