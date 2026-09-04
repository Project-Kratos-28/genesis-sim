#!/usr/bin/env python3
import math
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
 
# Wheelbase derived from athena_rover-6.urdf: front wheel x ~ -0.393,
# rear wheel x ~ +0.345, relative to base_link.
WHEELBASE = 0.738  # m
 
# Steering angles beyond this are clamped before computing wz, to avoid
# tan() blowing up near +-90 deg. The rover's own steer joints only go to
# about +-45 deg (0.79 rad) anyway, so this is a generous safety bound.
MAX_STEERING_ANGLE = 1.57  # rad (~74 deg)
 
 
class AckermannToTwist(Node):
    def __init__(self):
        super().__init__("ackermann_to_twist")
 
        self.declare_parameter("ackermann_topic", "/ackermann_cmd")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("stamped", True)
        self.declare_parameter("wheelbase", WHEELBASE)
 
        ackermann_topic = self.get_parameter("ackermann_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.stamped = self.get_parameter("stamped").value
        self.wheelbase = self.get_parameter("wheelbase").value
 
        msg_type = AckermannDriveStamped if self.stamped else AckermannDrive
        self.sub_ = self.create_subscription(
            msg_type, ackermann_topic, self.on_ackermann_cmd, 10
        )
        self.pub_ = self.create_publisher(Twist, cmd_vel_topic, 10)
 
        self.get_logger().info(
            f"ackermann_to_twist: {ackermann_topic}"
            f"({'AckermannDriveStamped' if self.stamped else 'AckermannDrive'}) "
            f"-> {cmd_vel_topic}, wheelbase={self.wheelbase:.3f} m"
        )
 
    def on_ackermann_cmd(self, msg):
        drive = msg.drive if self.stamped else msg
 
        speed = drive.speed
        angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, drive.steering_angle))
 
        wz = speed * math.tan(angle) / self.wheelbase
 
        out = Twist()
        out.linear.x = speed
        out.linear.y = 0.0
        out.angular.z = wz
        self.pub_.publish(out)

 
def main():
    rclpy.init()
    node = AckermannToTwist()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()