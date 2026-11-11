"""Simulated ZED2i: Genesis stereo camera pair + IMU, published as ROS 2.

All sensors are created before ``scene.build()`` and are attached to the rover
link that survives ``merge_fixed_links`` (``base_footprint``). Pose, baseline,
resolution and intrinsics are configured in :mod:`genesis_bridge.config` to
match the real ZED2i HD720 stream, so a VIO stack consuming these topics can be
moved to the physical camera without reconfiguration.

Topic / frame names follow the ZED ROS 2 wrapper:

==============================  ============================================
topic                           contents
==============================  ============================================
``left/image_rect_color``       rectified RGB, ``rgb8``
``right/image_rect_color``      rectified RGB, ``rgb8``
``left/camera_info``            pinhole intrinsics, ``P`` carries the baseline
``right/camera_info``           pinhole intrinsics, ``P[0,3] = -fx * baseline``
``imu/data_raw``                accel + gyro, no orientation (raw)
==============================  ============================================

Orientation is *not* estimated here: as on the real camera, ``imu/data`` is
produced by filtering ``imu/data_raw`` (``imu_filter_madgwick`` in
``rtabmap_vio.launch.py``), and that is the topic VIO consumes.

Genesis renders a pinhole model with no lens distortion, so the images are
already rectified and ``D`` is zero.
"""

import numpy as np
from sensor_msgs.msg import CameraInfo, Image, Imu

import genesis as gs

from . import config

from genesis.vis.rasterizer_context import RasterizerContext

_orig_update_frustum = RasterizerContext.update_camera_frustum

def _update_frustum_if_known(self, camera):
    if camera.uid is self.frustum_nodes:
        _orig_update_frustum(self, camera)

RasterizerContext.update_camera_frustum = _update_frustum_if_known


class ZedSensors:
    def __init__(self, node, scene, rover):
        self.node = node
        self.link = rover.get_link(config.ZED_ATTACH_LINK)

        # Genesis sensors must exist before scene.build()
        self.cameras = {
            "left": scene.add_sensor(self._camera_options(rover, +config.ZED_CAMERA_Y)),
            "right": scene.add_sensor(self._camera_options(rover, -config.ZED_CAMERA_Y)),
        }
        self.imu = scene.add_sensor(self._imu_options(rover))

        self._create_publishers()

        # Sim-time bookkeeping; initialised to trigger on the first step.
        # reset_time() is called by SimulationRunner after _settle() to align
        # these baselines with the final sim_time offset so rate-limiting works
        # correctly from the very first published stamp.
        self._last_image_time = -np.inf
        self._last_imu_time = -np.inf

    # --- Genesis setup ------------------------------------------------------
    def _camera_options(self, rover, y):
        """Pinhole camera at the URDF optical frame, looking along +X."""
        pos = (config.ZED_CAMERA_X, y, config.ZED_CAMERA_Z)
        # Genesis cameras look down their local -Z, so `lookat` is a point
        # ahead of `pos` in the link frame.
        lookat = (config.ZED_CAMERA_X + 1.0, y, config.ZED_CAMERA_Z)
        return gs.sensors.RasterizerCameraOptions(
            res=(config.ZED_WIDTH, config.ZED_HEIGHT),
            fov=config.ZED_VERTICAL_FOV,
            near=config.ZED_NEAR,
            pos=pos,
            lookat=lookat,
            up=(0.0, 0.0, 1.0),
            entity_idx=rover.idx,
            link_idx_local=self.link.idx_local,
        )

    def _imu_options(self, rover):
        n_acc, n_gyro = config.ZED_IMU_ACC_NOISE, config.ZED_IMU_GYRO_NOISE
        return gs.sensors.IMU(
            entity_idx=rover.idx,
            link_idx_local=self.link.idx_local,
            pos_offset=config.ZED_IMU_POS,
            euler_offset=(0.0, 0.0, 0.0),
            acc_noise=(n_acc, n_acc, n_acc),
            gyro_noise=(n_gyro, n_gyro, n_gyro),
            acc_bias=config.ZED_IMU_ACC_BIAS,
            gyro_bias=config.ZED_IMU_GYRO_BIAS,
        )

    # --- ROS setup ----------------------------------------------------------
    def _create_publishers(self):
        mk = self.node.create_publisher
        self.pub_left = mk(Image, config.TOPIC_ZED_LEFT_IMAGE, 5)
        self.pub_right = mk(Image, config.TOPIC_ZED_RIGHT_IMAGE, 5)
        self.pub_left_info = mk(CameraInfo, config.TOPIC_ZED_LEFT_INFO, 5)
        self.pub_right_info = mk(CameraInfo, config.TOPIC_ZED_RIGHT_INFO, 5)
        self.pub_imu = mk(Imu, config.TOPIC_ZED_IMU_RAW, 50)

    # --- Time reset ---------------------------------------------------------
    def reset_time(self, sim_time):
        """Align rate-limiter baselines after the clock offset is applied.

        Called by SimulationRunner after _settle() / apply_clock_offset() so that
        the first image and IMU publishes are separated from sim_time by exactly
        one period (not by a huge gap caused by _last_*_time starting at -inf).
        """
        self._last_image_time = sim_time - 1.0 / config.ZED_IMAGE_RATE
        self._last_imu_time = sim_time - 1.0 / config.ZED_IMU_RATE

    # --- Publishing ---------------------------------------------------------
    def publish(self, sim_time):
        """Publish whatever is due at `sim_time` (seconds of simulated time)."""
        if sim_time - self._last_imu_time >= 1.0 / config.ZED_IMU_RATE:
            self._publish_imu()
            self._last_imu_time = sim_time

        # Rendering a 1280x720 stereo pair costs ~40 ms (and ~25x more while the
        # interactive viewer is open), so only pay for it when something is
        # actually consuming the images. `camera_info` follows the images; the
        # IMU is cheap and streams unconditionally.
        if self.pub_left.get_subscription_count() or self.pub_right.get_subscription_count():
            if sim_time - self._last_image_time >= 1.0 / config.ZED_IMAGE_RATE:
                self._publish_stereo()
                self._last_image_time = sim_time

    def _publish_stereo(self):
        stamp = self.node.stamp()
        for side, frame, info_pub in (
            ("left", config.ZED_LEFT_FRAME, self.pub_left_info),
            ("right", config.ZED_RIGHT_FRAME, self.pub_right_info),
        ):
            rgb = self.cameras[side].read().rgb.cpu().numpy()
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = frame
            msg.height, msg.width = rgb.shape[0], rgb.shape[1]
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = rgb.tobytes()
            (self.pub_left if side == "left" else self.pub_right).publish(msg)

            info = self._camera_info(stamp, frame, right=(side == "right"))
            info_pub.publish(info)

    def _camera_info(self, stamp, frame, right):
        fx, fy = config.ZED_FX, config.ZED_FY
        cx, cy = config.ZED_CX, config.ZED_CY

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.width = config.ZED_WIDTH
        msg.height = config.ZED_HEIGHT
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        tx = -fx * config.ZED_BASELINE if right else 0.0
        msg.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _publish_imu(self):
        data = self.imu.read()

        msg = Imu()
        msg.header.stamp = self.node.stamp()
        msg.header.frame_id = config.ZED_IMU_FRAME

        # Raw (unfiltered) IMU, exactly like the wrapper's `imu/data_raw`: no
        # orientation is estimated here, so `orientation_covariance[0] = -1`
        # marks it unavailable and `imu_filter_madgwick` turns this stream into
        # `imu/data` (which is what the VIO stack consumes).
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        gyro = data.ang_vel.cpu().numpy()
        acc = data.lin_acc.cpu().numpy()
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            float(gyro[0]), float(gyro[1]), float(gyro[2])
        )
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            float(acc[0]), float(acc[1]), float(acc[2])
        )

        # Row-major 3x3 diagonals.
        g2 = config.ZED_IMU_GYRO_NOISE ** 2
        a2 = config.ZED_IMU_ACC_NOISE ** 2
        msg.angular_velocity_covariance = [g2, 0.0, 0.0, 0.0, g2, 0.0, 0.0, 0.0, g2]
        msg.linear_acceleration_covariance = [a2, 0.0, 0.0, 0.0, a2, 0.0, 0.0, 0.0, a2]

        self.pub_imu.publish(msg)
