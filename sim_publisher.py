import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from cev_msgs.msg import Waypoint
from typing import List, Tuple


def yaw_to_quaternion(yaw: float):
    """Return quaternion (x,y,z,w) for a yaw angle."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class VehiclePublisher(Node):
    def __init__(self):
        super().__init__('sim_publisher')
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'sim_pose', 10)
        self.state_publisher_ = self.create_publisher(Waypoint, 'sim_state', 10)
        occ_qos = QoSProfile(depth=1)
        occ_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        occ_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.occ_publisher_ = self.create_publisher(OccupancyGrid, 'sim_occupancy', occ_qos)

    def publish_pose(self, x, y, theta, v, steering_angle):
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

        self.pose_publisher_.publish(msg)
        self.get_logger().info(f'Published sim pose stamped: x={x}, y={y}, theta={theta}')

        # publish Waypoint
        w = Waypoint()
        w.x = float(x)
        w.y = float(y)
        w.v = float(v)
        w.tau = float(steering_angle)
        w.theta = float(theta)
        self.state_publisher_.publish(w)
        self.get_logger().info(f'Published sim state: x={x}, y={y}, v={v}, tau={steering_angle}, theta={theta}')

    # Occupancy grid helpers
    def _point_in_polygon(self, x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
        inside = False
        n = len(polygon)
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            intersect = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
            if intersect:
                inside = not inside
            j = i
        return inside

    def _point_in_circle(self, x: float, y: float, cx: float, cy: float, r: float) -> bool:
        return (x - cx) ** 2 + (y - cy) ** 2 <= r * r

    def publish_occupancy_grid(self, obstacles, width_m=20.0, height_m=20.0, resolution=0.1, frame_id='map'):
        nx = int(math.ceil(width_m / resolution))
        ny = int(math.ceil(height_m / resolution))
        origin_x = -width_m / 2.0
        origin_y = -height_m / 2.0

        grid = [-1] * (nx * ny)

        for iy in range(ny):
            for ix in range(nx):
                cx = origin_x + (ix + 0.5) * resolution
                cy = origin_y + (iy + 0.5) * resolution
                occupied = False
                for obs in obstacles:
                    if hasattr(obs, 'radius'):
                        if self._point_in_circle(cx, cy, obs.x, obs.y, obs.radius):
                            occupied = True
                            break
                    elif hasattr(obs, 'vertices'):
                        if self._point_in_polygon(cx, cy, obs.vertices):
                            occupied = True
                            break
                idx = ix + iy * nx
                grid[idx] = 100 if occupied else 0

        occ = OccupancyGrid()
        occ.header = Header()
        occ.header.stamp = self.get_clock().now().to_msg()
        occ.header.frame_id = frame_id
        info = MapMetaData()
        info.map_load_time = occ.header.stamp
        info.resolution = float(resolution)
        info.width = nx
        info.height = ny
        info.origin.position.x = float(origin_x)
        info.origin.position.y = float(origin_y)
        info.origin.position.z = 0.0
        info.origin.orientation.x = 0.0
        info.origin.orientation.y = 0.0
        info.origin.orientation.z = 0.0
        info.origin.orientation.w = 1.0
        occ.info = info
        occ.data = grid
        self.occ_publisher_.publish(occ)
        self.get_logger().debug(f'Published OccupancyGrid: size={nx}x{ny}, res={resolution}')