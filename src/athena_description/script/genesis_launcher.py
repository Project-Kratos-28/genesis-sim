#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import genesis as gs
import os
from sensor_msgs.msg import Image
import numpy as np

class GenesisZedBridge(Node):
    def __init__(self):
        super().__init__("genesis_zed_bridge")
        self.rgb_pub = self.create_publisher(Image, '/zed2i/zed_node/rgb/image_rect_color', 10)
        self.depth_pub = self.create_publisher(Image, '/zed2i/zed_node/depth/depth_registered', 10)


        # gs setup
        gs.init(backend=gs.gpu)
        self.scene = gs.Scene(
            show_viewer=True
        )
        self.scene.add_entity(gs.morphs.Plane())
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=os.path.expanduser("~/Desktop/kratos/src/athena_description/urdf/athena_rover-6.urdf"), 
                fixed=False,
                links_to_keep=['zed2i_left_camera_frame_optical', 'zed2i_right_camera_frame_optical'],
            )
        )
        self.cam = self.scene.add_camera(res=(1280, 720), fov=90, GUI=False)
        self.scene.build()


        zed_link = self.robot.get_link('zed2i_left_camera_frame_optical')
        self.cam.attach(zed_link, offset_T=np.eye(4))

        self.timer = self.create_timer(1.0 / 30.0, self.step_and_publish)

    def numpy_to_imgmsg(self, arr, encoding, frame_id, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = arr.shape[0]
        msg.width = arr.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = arr.shape[1] * arr.itemsize * (arr.shape[2] if arr.ndim == 3 else 1)
        msg.data = arr.tobytes()
        return msg

    def step_and_publish(self):
        self.scene.step()
        rgb, depth, _, _ = self.cam.render(rgb=True, depth=True)

        stamp = self.get_clock().now().to_msg()

        rgb_msg = self.numpy_to_imgmsg(rgb.astype(np.uint8), 'rgb8', 'zed2i_left_camera_optical_frame', stamp)
        self.rgb_pub.publish(rgb_msg)

        depth_msg = self.numpy_to_imgmsg(depth.astype(np.float32), '32FC1', 'zed2i_left_camera_optical_frame', stamp)
        self.depth_pub.publish(depth_msg)



def main() :
    rclpy.init()
    node = GenesisZedBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
