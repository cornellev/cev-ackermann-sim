import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Header
from cev_msgs.msg import Waypoint, Trajectory
from tf2_ros import StaticTransformBroadcaster
from typing import List, Tuple


def yaw_to_quaternion(yaw: float):
    """Return quaternion (x,y,z,w) for a yaw angle."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class VehiclePublisher(Node):
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(VehiclePublisher, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
            
        super().__init__('sim_publisher', start_parameter_services=False)
        self._initialized = True
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'sim_pose', 10)
        self.state_publisher_ = self.create_publisher(Waypoint, 'sim_state', 10)
        occ_qos = QoSProfile(depth=1)
        occ_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        occ_qos.reliability = QoSReliabilityPolicy.RELIABLE
        # publish occupancy grid for simulator viewers and for planner
        self.occ_publisher_ = self.create_publisher(OccupancyGrid, 'sim_occupancy', occ_qos)
        self.map_publisher_ = self.create_publisher(OccupancyGrid, 'map', occ_qos)

        # odometry publisher that the planner listens to
        self.odom_publisher_ = self.create_publisher(Odometry, '/odometry/filtered', 10)

        # TF static broadcaster: publish identity map -> odom so planner can transform
        try:
            self.static_broadcaster = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            # send static transform once
            self.static_broadcaster.sendTransform(t)
        except Exception:
            # If tf2_ros not available or something fails, we continue without broadcasting
            self.get_logger().warning('Static TF broadcaster unavailable; planner may not receive transforms')

        # Subscription to planner trajectories so the simulator can follow them
        self.latest_trajectory_msg = None
        try:
            self.trajectory_sub = self.create_subscription(Trajectory, 'trajectory', self._trajectory_callback, 10)
        except Exception:
            # If message isn't available or subscriber can't be created, ignore
            self.get_logger().warning('Could not create trajectory subscriber; simulator will not follow planner')
        # Subscription to external follower drive commands (ackermann).
        # Import AckermannDrive at runtime so this module can be imported before
        # the ackermann_msgs Python package is available (e.g., during initial colcon build).
        self.latest_ack_msg = None
        self.ack_sub_exists = False
        self._last_ack_time = None
        try:
            from ackermann_msgs.msg import AckermannDrive
            try:
                self.ack_sub = self.create_subscription(AckermannDrive, 'rc_movement_msg', self._ack_callback, 10)
                self.ack_sub_exists = True
            except Exception:
                self.get_logger().warning('Could not create ackermann subscriber; simulator will not accept external drive commands')
        except Exception:
            # ackermann_msgs not yet available; continue without ack subscription
            self.get_logger().debug('ackermann_msgs not available at import time; skipping ack subscription')
        # Publisher for target waypoint that planner listens to
        try:
            self.target_publisher_ = self.create_publisher(Waypoint, 'target', 10)
        except Exception:
            self.get_logger().warning('Could not create target publisher')

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
        # self.get_logger().info(f'Published sim pose stamped: x={x}, y={y}, theta={theta}')

        # publish Waypoint
        w = Waypoint()
        w.x = float(x)
        w.y = float(y)
        w.v = float(v)
        w.tau = float(steering_angle)
        w.theta = float(theta)
        self.state_publisher_.publish(w)
        # self.get_logger().info(f'Published sim state: x={x}, y={y}, v={v}, tau={steering_angle}, theta={theta}')

    def publish_odometry(self, x, y, theta, v):
        """Publish a nav_msgs/Odometry message to /odometry/filtered for the planner."""
        try:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            odom.pose.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quaternion(float(theta))
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = float(v)
            self.odom_publisher_.publish(odom)
        except Exception as e:
            self.get_logger().debug(f'Failed to publish odometry: {e}')

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
        # Publish both simulator occupancy and planner-facing map topic
        try:
            occ.header.frame_id = frame_id
            self.occ_publisher_.publish(occ)
        except Exception:
            pass

        try:
            occ.header.frame_id = frame_id
            self.map_publisher_.publish(occ)
            self.get_logger().debug(f'Published OccupancyGrid on "map": size={nx}x{ny}, res={resolution}')
        except Exception:
            self.get_logger().debug('Failed to publish OccupancyGrid on "map"')

    def _trajectory_callback(self, msg: Trajectory):
        # store last received trajectory message for simulator to consume
        self.latest_trajectory_msg = msg
        self.get_logger().info('Received trajectory from planner')

    def _ack_callback(self, msg):
        # Store last ackermann drive message from external follower
        self.latest_ack_msg = msg
            # store a wall-clock timestamp so callers can know how recent the message is
        self._last_ack_time = time.time()
        self.get_logger().info('Received AckermannDrive from external follower')

    def get_latest_ack(self):
        return self.latest_ack_msg

    def ack_age_seconds(self):
        """Return age in seconds of last received ack message, or None if unknown."""
        if getattr(self, '_last_ack_time', None) is None:
            return None
        return time.time() - self._last_ack_time

    def is_follower_connected(self) -> bool:
        """Return True if an ack subscriber was created or we have recently received an ack message."""
        if getattr(self, 'ack_sub_exists', False):
            return True
        return self.latest_ack_msg is not None

    def publish_target(self, x: float, y: float, v: float = 0.0, tau: float = 0.0, theta: float = 0.0):
        try:
            w = Waypoint()
            w.x = float(x)
            w.y = float(y)
            w.v = float(v)
            w.tau = float(tau)
            w.theta = float(theta)
            self.target_publisher_.publish(w)
            # self.get_logger().info(f'Published target waypoint: x={x}, y={y}, v={v}, tau={tau}, theta={theta}')
        except Exception as e:
            self.get_logger().debug(f'Failed to publish target: {e}')