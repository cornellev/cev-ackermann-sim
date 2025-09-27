import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw: float):
    """Return quaternion (x,y,z,w) for a yaw angle."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class VehiclePublisher(Node):
    def __init__(self):
        super().__init__('sim_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'sim_pose', 10)

    def publish_pose(self, x, y, theta):
        msg = PoseStamped()
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Position
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        # Orientation from yaw
        qx, qy, qz, qw = yaw_to_quaternion(float(theta))
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published sim pose stamped: x={x}, y={y}, theta={theta}')
