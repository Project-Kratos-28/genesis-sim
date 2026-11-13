import sys

import genesis as gs
import rclpy

from . import config
from .bridge_node import BridgeAlreadyRunning, GenesisBridge
from .livox_lidar import LivoxLidar
from .penetration_debug import WheelPenetrationMonitor
from .publishers import BridgePublishers
from .rover_controller import RoverController
from .scene import build_scene
from .zed_sensors import ZedSensors


class SimulationRunner:
    def __init__(self):
        self.node = GenesisBridge()
        self.sim = build_scene(
            lambda scene, rover: (
                ZedSensors(self.node, scene, rover),
                LivoxLidar(self.node, scene, rover),
            ),
            show_viewer=self.node.show_viewer,
        )
        self.controller = RoverController(self.sim.rover)
        self.penetration = WheelPenetrationMonitor(self.sim.rover, self.sim.terrain)
        self.pubs = BridgePublishers(self.node)
        self.zed, self.lidar = self.sim.sensors
        self.step_count = 0
        self._settle()

    def _settle(self):
        """Step the physics before the ROS clock starts.

        The rover spawns above the terrain, and during the drop the IMU sees free
        fall -- which breaks gravity/attitude initialisation for the VIO stack.
        Nothing is published during these steps, so /clock simply starts at the
        settled state.

        apply_clock_offset() is called after the loop to set sim_time to one tick
        past the last value persisted by a previous session, so downstream VIO
        nodes (stereo_odometry, rtabmap_viz) never see a backward jump even if
        the bridge is restarted while they keep running.
        """
        steer, wheel = self.node.compute_wheel_commands()
        for _ in range(config.SETTLE_STEPS):
            self.controller.apply(steer, wheel)
            self.sim.scene.step()

        # Lock the offset: sim_time will start just past the last persisted stamp.
        self.node.apply_clock_offset()
        # Synchronise the ZED sensor rate-limiters to the new sim_time baseline
        # so the first image/IMU publish is properly spaced from t=0 of this run.
        self.zed.reset_time(self.node.sim_time)
        self.lidar.reset_time(self.node.sim_time)

    def step(self):
        node, sim = self.node, self.sim

        steer, wheel = node.compute_wheel_commands()
        self.controller.apply(steer, wheel)
        sim.scene.step()
        self.penetration.update(node.get_logger())

        pos, quat = self.controller.pose()
        # self.pubs.publish_odom(pos, quat)

        node.advance_time(config.SIM_DT)
        self.pubs.publish_clock()
        self.pubs.publish_base_tf(pos, quat)
        self.zed.publish(node.sim_time)
        self.lidar.publish(node.sim_time)

        self._debug(wheel)

        self.step_count += 1
        rclpy.spin_once(node, timeout_sec=0)

    def _debug(self, commanded):
        n = config.DEBUG_PRINT_EVERY
        if n and self.step_count % n == 0:
            self.node.get_logger().info(
                f"wheel cmd: {commanded} | actual: {self.controller.wheel_velocities()}"
            )

    def run(self):
        while rclpy.ok():
            self.step()

    def shutdown(self):
        # Persist the final sim_time so the next session starts monotonically.
        self.penetration.log_summary(self.node.get_logger())
        self.node._persist_clock()
        self.node.destroy_node()
        self.node.release_instance_lock()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    rclpy.init()
    gs.init(backend=gs.gpu)
    try:
        runner = SimulationRunner()
    except BridgeAlreadyRunning as exc:
        print(f"\n[genesis_bridge] ERROR: {exc}\n", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1) from None
    try:
        runner.run()
    finally:
        runner.shutdown()
