from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
 
from steering import make_steering_mode
 
from . import config
 
 
class GenesisZedBridge(Node):
    def __init__(self):
        super().__init__("genesis_zed_bridge")
 
        self.sim_time = 0.0
        self.target_v = 0.0
        self.target_wz = 0.0
 
        self.create_subscription(Twist, config.TOPIC_CMD_VEL, self._on_cmd_vel, 10)
        self.create_subscription(String, config.TOPIC_STEERING_MODE, self._on_steering_mode, 10)
 
        self.declare_parameter("steering_mode", config.DEFAULT_STEERING_MODE)
        self.steering = self._init_steering(self.get_parameter("steering_mode").value)
 
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
 
    def stamp(self):
        sec = int(self.sim_time)
        return Time(sec=sec, nanosec=int((self.sim_time - sec) * 1e9))
