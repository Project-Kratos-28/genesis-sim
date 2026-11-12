"""Simulated Livox MID-360 LiDAR, published as a ROS 2 PointCloud2.

The scan is cast by Genesis against the scene's raycast geometry: the collision
geometry the physics uses (terrain + rover) *plus* the visual landmark boulders,
which opt in through `use_visual_raycasting` on their material (see
`scene.py::_add_visual_landmarks`). The boulders are still `collision=False`, so
they show up in the scan without affecting the dynamics; they use the
`Kinematic` material so their visual BVH is static and built only once. It is
published on the topic name used by ``livox_ros_driver2``:

===============================  ===========================================
topic                            contents
===============================  ===========================================
``/livox/lidar``                 ``sensor_msgs/PointCloud2`` (x, y, z,
                                 intensity), frame ``livox_mid360_link``
===============================  ===========================================

``livox_ros_driver2`` emits ``livox_ros_driver2/msg/CustomMsg`` by default and
``sensor_msgs/PointCloud2`` on the *same* topic when launched with
``xfer_format:=1``; this bridge publishes the PointCloud2 form. On hardware,
point the URDF/static transform at ``livox_mid360_link`` (or set the driver's
``frame_id`` to match) so the TF tree is identical to the one here.

Geometry mirrors the MID-360 datasheet: full 360 deg azimuth, -7 .. +52 deg
elevation, 0.1 m blind zone. The sample pattern is a uniform spherical grid
rather than the real non-repetitive rosette, so density is representative,
not identical. Ray hits along the -7 deg rings may include the rover's own
body and the ZED2i housing, exactly as a 360 deg unit mounted at that height
would see them; growing ``LIVOX_MIN_RANGE`` filters them if unwanted.

Scans are only published while something is subscribed, following the same
policy as the simulated ZED cameras (the raycast itself is owned by Genesis,
which refreshes its BVH each physics step regardless).
"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

import genesis as gs

from . import config


class LivoxLidar:
    def __init__(self, node, scene, rover):
        self.node = node
        self.link = rover.get_link(config.LIVOX_ATTACH_LINK)

        # Genesis sensors must exist before scene.build()
        self.lidar = scene.add_sensor(self._lidar_options(rover))

        self.pub = node.create_publisher(PointCloud2, config.TOPIC_LIVOX_POINTS, 5)

        # Sim-time bookkeeping; reset_time() re-bases this after _settle().
        self._last_scan_time = -np.inf

    # --- Genesis setup ------------------------------------------------------
    def _lidar_options(self, rover):
        pattern = gs.sensors.SphericalPattern(
            fov=(360.0, config.LIVOX_VERTICAL_FOV),
            n_points=(config.LIVOX_BEAMS_H, config.LIVOX_BEAMS_V),
        )
        return gs.sensors.Lidar(
            pattern=pattern,
            entity_idx=rover.idx,
            link_idx_local=self.link.idx_local,
            pos_offset=config.LIVOX_POS,
            euler_offset=(0.0, 0.0, 0.0),
            min_range=config.LIVOX_MIN_RANGE,
            max_range=config.LIVOX_MAX_RANGE,
            return_world_frame=False,
            return_points=True,
        )

    # --- Time reset ---------------------------------------------------------
    def reset_time(self, sim_time):
        """Align the rate-limiter baseline after the clock offset is applied."""
        self._last_scan_time = sim_time - 1.0 / config.LIVOX_SCAN_RATE

    # --- Publishing ---------------------------------------------------------
    def publish(self, sim_time):
        """Publish a scan if one is due and a consumer is listening."""
        if not self.pub.get_subscription_count():
            return
        if sim_time - self._last_scan_time < 1.0 / config.LIVOX_SCAN_RATE:
            return
        self._last_scan_time = sim_time
        self._publish_scan()

    def _publish_scan(self):
        data = self.lidar.read()
        points = data.points.reshape(-1, 3).cpu().numpy().astype(np.float64)
        ranges = data.distances.reshape(-1).cpu().numpy().astype(np.float64)

        # Misses come back as no_hit_value (== max_range) with points at the
        # ray origin; drop them and anything inside the blind zone.
        valid = np.isfinite(ranges) & (ranges > config.LIVOX_MIN_RANGE) & (
            ranges < config.LIVOX_MAX_RANGE
        )
        points = points[valid]
        ranges = ranges[valid]

        if points.shape[0] == 0:
            return

        if config.LIVOX_RANGE_NOISE > 0.0:
            noise = np.random.normal(0.0, config.LIVOX_RANGE_NOISE, ranges.shape[0])
            scale = (ranges + noise) / np.maximum(ranges, 1e-6)
            points = points * scale[:, None]

        intensity = np.clip(1.0 / (1.0 + (ranges / 10.0) ** 2), 0.05, 1.0)

        self.pub.publish(
            self._to_msg(points.astype(np.float32), intensity.astype(np.float32))
        )

    def _to_msg(self, points, intensity):
        n = points.shape[0]
        record = np.empty((n, 4), dtype=np.float32)
        record[:, :3] = points
        record[:, 3] = intensity

        msg = PointCloud2()
        msg.header.stamp = self.node.stamp()
        msg.header.frame_id = config.LIVOX_FRAME
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = False
        msg.data = record.tobytes()
        return msg
