#!/usr/bin/env python3

import numpy as np, rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

class Chk(Node):
    def __init__(s):
        super().__init__('imu_check')
        s.buf = Buffer(); TransformListener(s.buf, s)
        s.q = None; s.n = 0
        s.create_subscription(Odometry, '/odom', lambda m: setattr(s, 'q', m.pose.pose.orientation), 10)
        s.create_subscription(Imu, '/imu/data_raw', s.on_imu, 10)
    def on_imu(s, m):
        s.n += 1
        if s.n % 100 or s.q is None: return
        try:
            t = s.buf.lookup_transform('base_footprint', 'zed2i_camera_center', Time()).transform.rotation
        except Exception as e:
            print('no tf yet', e); return
        Rg = R.from_quat([s.q.x, s.q.y, s.q.z, s.q.w])
        Rb = R.from_quat([t.x, t.y, t.z, t.w])
        exp = (Rg * Rb).inv().apply([0, 0, 9.81])      # accel at rest, in IMU frame
        a = m.linear_acceleration
        meas = np.array([a.x, a.y, a.z])
        c = meas @ exp / (np.linalg.norm(meas) * np.linalg.norm(exp) + 1e-9)
        print(f"measured {np.round(meas,2)} |{np.linalg.norm(meas):.2f}|  "
              f"expected {np.round(exp,2)}  angle {np.degrees(np.arccos(np.clip(c,-1,1))):.1f} deg")

rclpy.init(); rclpy.spin(Chk())