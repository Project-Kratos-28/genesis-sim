from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMsg
from tf2_ros import TransformBroadcaster
 
from . import config


class BridgePublishers:
    def __init__(self, node):
        self.node = node
        self.tf_broadcaster = TransformBroadcaster(node)
 
        mk = node.create_publisher
        self.clock = mk(ClockMsg, config.TOPIC_CLOCK, 10)
        # self.odom = mk(Odometry, config.TOPIC_ODOM, 10)
 
    # --- state / time -------------------------------------------------------
    def publish_clock(self):
        msg = ClockMsg()
        msg.clock = self.node.stamp()
        self.clock.publish(msg)
 
    def publish_base_tf(self, pos, quat):
        t = TransformStamped()
        t.header.stamp = self.node.stamp()
        t.header.frame_id = config.MAP_FRAME
        t.child_frame_id = config.BASE_FOOTPRINT_FRAME
        self._fill_pose(t.transform.translation, t.transform.rotation, pos, quat)
        self.tf_broadcaster.sendTransform(t)
 
    def publish_odom(self, pos, quat):
        msg = Odometry()
        msg.header.stamp = self.node.stamp()
        msg.header.frame_id = config.MAP_FRAME
        msg.child_frame_id = config.ODOM_CHILD_FRAME
        p = msg.pose.pose
        self._fill_pose(p.position, p.orientation, pos, quat)
        # self.odom.publish(msg)
 
    @staticmethod
    def _fill_pose(trans, rot, pos, quat):
        trans.x, trans.y, trans.z = float(pos[0]), float(pos[1]), float(pos[2])
        # Genesis quats are [w, x, y, z]; ROS wants x, y, z, w.
        rot.x, rot.y, rot.z, rot.w = (float(quat[1]), float(quat[2]),
                                      float(quat[3]), float(quat[0]))
