import os

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from steering import make_steering_mode

from . import config


class GenesisBridge(Node):
    def __init__(self):
        super().__init__("genesis_bridge")

        self.sim_time = 0.0
        self.target_v = 0.0
        self.target_wz = 0.0

        self.create_subscription(Twist, config.TOPIC_CMD_VEL, self._on_cmd_vel, 10)
        self.create_subscription(String, config.TOPIC_STEERING_MODE, self._on_steering_mode, 10)

        self.declare_parameter("steering_mode", config.DEFAULT_STEERING_MODE)
        self.steering = self._init_steering(self.get_parameter("steering_mode").value)

        # The interactive viewer and the ZED render cameras fight over the GPU:
        # sensor reads are ~25x slower while the viewer window is open (measured
        # 8.9 vs 97.5 steps/s), which starves the VIO pipeline. Exposed as a
        # parameter so a launch can disable the window (rtabmap_vio does).
        self.declare_parameter("show_viewer", bool(config.SHOW_VIEWER))
        _viewer = self.get_parameter("show_viewer").value
        self.show_viewer = (
            _viewer if isinstance(_viewer, bool)
            else str(_viewer).strip().lower() in ("1", "true", "yes", "on")
        )

        # --- Monotonic sim-time offset ------------------------------------------
        # On restart the downstream VIO nodes (stereo_odometry, rtabmap_viz) keep
        # running and remember the last /clock value they saw.  If we restart
        # sim_time from 0 they detect a backward jump and flush their TF buffers,
        # breaking odometry.
        #
        # Strategy: persist the last sim_time to a small file at every publish so
        # that on restart we can read it back directly — no QoS race, no waiting
        # for a live /clock subscriber to echo the value back to us.
        self._clock_offset = self._load_persisted_clock()

    # --- persistence helpers ---------------------------------------------------

    @staticmethod
    def _load_persisted_clock() -> float:
        """Return the last persisted sim_time, or 0.0 if no file exists."""
        path = config.CLOCK_PERSIST_PATH
        try:
            with open(path, "r") as fh:
                return float(fh.read().strip())
        except (OSError, ValueError):
            return 0.0

    def _persist_clock(self) -> None:
        """Overwrite the persist file with the current sim_time."""
        path = config.CLOCK_PERSIST_PATH
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as fh:
                fh.write(f"{self.sim_time:.9f}\n")
        except OSError as exc:
            self.get_logger().warn(f"Could not persist clock: {exc}")

    def apply_clock_offset(self):
        """Call this after _settle() to set sim_time past the last seen stamp."""
        # Start one SIM_DT tick past whatever the previous session published last.
        # This guarantees the very first stamp we broadcast is strictly greater
        # than any stamp already in the downstream TF / odometry buffers.
        self.sim_time = self._clock_offset + config.SIM_DT
        if self._clock_offset > 0.0:
            self.get_logger().info(
                f"Clock offset applied: sim_time starts at {self.sim_time:.3f}s "
                f"(previous session ended at ~{self._clock_offset:.3f}s)"
            )

    ## Steering
    def _init_steering(self, requested):
        try:
            mode = make_steering_mode(requested)
        except ValueError as e:
            self.get_logger().warn(f"{e}; falling back to '{config.DEFAULT_STEERING_MODE}'")
            mode = make_steering_mode(config.DEFAULT_STEERING_MODE)
        self.get_logger().info(f"Steering mode: {mode.name}")
        return mode

    def _on_steering_mode(self, msg):
        requested = msg.data.strip().lower()
        if requested == self.steering.name:
            return
        try:
            self.steering = make_steering_mode(requested)
            self.get_logger().info(f"Switched steering mode -> {self.steering.name}")
        except ValueError as e:
            self.get_logger().warn(str(e))

    def _on_cmd_vel(self, msg):
        self.target_v = msg.linear.x
        self.target_wz = msg.angular.z

    def compute_wheel_commands(self):
        return self.steering.compute(self.target_v, self.target_wz)

    ## Time
    def advance_time(self, dt):
        self.sim_time += dt
        # Persist every ~1 simulated second (100 steps × SIM_DT 0.01 s) so a
        # clean restart can continue the clock monotonically without relying on
        # a live /clock subscriber callback (which is unreliable across process
        # restarts because /clock uses VOLATILE QoS and there is no retained
        # last-value cache to read back from).
        if int(self.sim_time / dt) % 100 == 0:
            self._persist_clock()

    def stamp(self):
        t = self.sim_time
        sec = int(t)
        nanosec = int(round((t - sec) * 1e9))
        # Guard against float rounding nanosec up to 1_000_000_000.
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec = 0
        return Time(sec=sec, nanosec=nanosec)
