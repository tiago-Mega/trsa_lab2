"""Documented wheel-only odometry interface: republishes /wheel/odom as /odom and owns odom -> base_footprint in wheel mode."""
import copy
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

class WheelOdomInterface(Node):
    def __init__(self):
        super().__init__('wheel_odom_interface')
        self.pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf = TransformBroadcaster(self)
        self.path = deque(maxlen=1500)
        self.last_time = None
        self.path_pub = self.create_publisher(Path, '/fusion/wheel_path', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.info = self.create_publisher(String, '/trsa/runtime_info', qos)
        self.mode_pub = self.create_publisher(String, '/fusion/selected_source', qos)
        msg = String(); msg.data = 'wheel'; self.mode_pub.publish(msg)
        runtime = String(); runtime.data = '{"revision":"lab2-2026.6","odometry_source":"wheel"}'; self.info.publish(runtime)
        self.create_subscription(Odometry, '/wheel/odom', self.receive, qos_profile_sensor_data)
        self.get_logger().info('Publishing /odom and odom -> base_footprint from /wheel/odom')

    def receive(self, msg):
        if (msg.header.frame_id, msg.child_frame_id) != ('odom', 'base_footprint'):
            return
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        vals = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in vals):
            return
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if abs(norm - 1.0) > 0.02:
            return
        self.pub.publish(msg)
        tf = TransformStamped(); tf.header = copy.deepcopy(msg.header)
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = p.x; tf.transform.translation.y = p.y; tf.transform.translation.z = p.z
        tf.transform.rotation = copy.deepcopy(q)
        self.tf.sendTransform(tf)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.last_time is not None and t < self.last_time:
            self.path.clear(); self.last_time = None
        if self.last_time is None or t - self.last_time >= 0.5:
            ps = PoseStamped(); ps.header = copy.deepcopy(msg.header); ps.pose = copy.deepcopy(msg.pose.pose)
            self.path.append(ps); self.last_time = t
            path = Path(); path.header = copy.deepcopy(msg.header); path.poses = list(self.path)
            self.path_pub.publish(path)

def main():
    rclpy.init(); node = WheelOdomInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
