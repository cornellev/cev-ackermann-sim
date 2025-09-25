import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class VehiclePublisher(Node):
    def __init__(self):
        super().__init__('sim_publisher')
        self.publisher_ = self.create_publisher(Pose2D, 'sim_pose', 10)

    def publish_pose(self, x, y, theta):
        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = theta
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published sim pose: x={x}, y={y}, theta={theta}')
