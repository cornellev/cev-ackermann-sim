import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Header, Float32MultiArray
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

        # Publisher for simulator-generated trajectory (list of Waypoint)
        try:
            self.trajectory_publisher_ = self.create_publisher(Trajectory, 'igvc_waypoints', 10)
        except Exception:
            self.get_logger().warning('Could not create igvc_waypoints publisher')

        # Publisher for lane centerline visualization
        try:
            self.lane_publisher_ = self.create_publisher(Trajectory, 'igvc_lane', 10)
        except Exception:
            self.get_logger().warning('Could not create igvc_lane publisher')

        # Live tuning for lane-following MPC cost weights
        self.cost_weights_pub = None
        try:
            self.cost_weights_pub = self.create_publisher(Float32MultiArray, 'lane_cost_weights', 10)
        except Exception:
            self.get_logger().warning('Could not create lane_cost_weights publisher')

        # Subscriptions for planner-generated costmap data (raw float values)
        self.latest_costmap_data = None
        self.latest_costmap_meta = None
        self._costmap_seq = 0
        try:
            costmap_qos = QoSProfile(depth=1)
            costmap_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            costmap_qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.costmap_meta_sub = self.create_subscription(
                MapMetaData, 'local_costmap_meta', self._costmap_meta_callback, costmap_qos)
            self.costmap_raw_sub = self.create_subscription(
                Float32MultiArray, 'local_costmap_raw', self._costmap_raw_callback, costmap_qos)
        except Exception:
            self.get_logger().warning('Could not create costmap subscriptions')


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

    def publish_lane_cost_weights(self, weights, target_vel):
        """
        Publish MPC lane-following cost weights for live tuning.
        Ordering matches planner expectation:
        [w_along_track, w_cte, w_costmap, obs_threshold, cte_threshold, along_threshold, target_vel]
        """
        try:
            if self.cost_weights_pub is None:
                return
            msg = Float32MultiArray()
            msg.data = [
                float(weights.get("along_track", 0.0)),
                float(weights.get("cte", 0.0)),
                float(weights.get("costmap", 0.0)),
                float(weights.get("obs_threshold", 0.0)),
                float(weights.get("cte_threshold", 0.0)),
                float(weights.get("along_threshold", 0.0)),
                float(target_vel),
            ]
            self.cost_weights_pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug(f'Failed to publish lane_cost_weights: {exc}')


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

    def publish_occupancy_grid(self, obstacles, width_m=30.0, height_m=30.0, resolution=0.1, frame_id='map'):
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
        # self.get_logger().info('Received AckermannDrive from external follower')

    def _costmap_meta_callback(self, msg):
        self.latest_costmap_meta = msg
        self._costmap_seq += 1

    def _costmap_raw_callback(self, msg):
        self.latest_costmap_data = msg
        self._costmap_seq += 1

    def costmap_seq(self):
        return self._costmap_seq

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
            w.cte = 0.0
            w.along_track = 0.0
            w.costmap_cost = 0.0
            w.cte_bad = 0.0
            w.obs_bad = 0.0
            w.along_track_penalty = 0.0
            self.target_publisher_.publish(w)
            # self.get_logger().info(f'Published target waypoint: x={x}, y={y}, v={v}, tau={tau}, theta={theta}')
        except Exception as e:
            self.get_logger().debug(f'Failed to publish target: {e}')

    def _waypoints_to_trajectory(self, waypoints: list) -> Trajectory:
        """Convert a heterogenous waypoint list into a Trajectory message."""
        tmsg = Trajectory()
        tmsg.waypoints = []
        for p in waypoints or []:
            w = Waypoint()
            try:
                if isinstance(p, (list, tuple)):
                    w.x = float(p[0])
                    w.y = float(p[1])
                elif isinstance(p, dict):
                    w.x = float(p.get('x', 0.0))
                    w.y = float(p.get('y', 0.0))
                else:
                    w.x = float(getattr(p, 'x', 0.0))
                    w.y = float(getattr(p, 'y', 0.0))
            except Exception:
                continue
            for attr in ('v', 'tau', 'theta'):
                try:
                    if isinstance(p, dict):
                        val = p.get(attr, None)
                    else:
                        val = getattr(p, attr, None)
                    if val is not None:
                        setattr(w, attr, float(val))
                except Exception:
                    pass
            w.cte = 0.0
            w.along_track = 0.0
            w.costmap_cost = 0.0
            w.cte_bad = 0.0
            w.obs_bad = 0.0
            w.along_track_penalty = 0.0
            tmsg.waypoints.append(w)
        return tmsg

    def publish_trajectory(self, waypoints: list):
        """Publish a list of waypoints as a Trajectory message on 'igvc_waypoints'."""
        try:
            if not hasattr(self, 'trajectory_publisher_'):
                return
            tmsg = self._waypoints_to_trajectory(waypoints)
            self.trajectory_publisher_.publish(tmsg)
            self.get_logger().debug(f'Published igvc_waypoints with {len(tmsg.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().debug(f'Failed to publish sim trajectory: {e}')

    def publish_lane_centerline(self, lane_points: list):
        """Publish the lane centerline on the /igvc_lane topic."""
        try:
            if not hasattr(self, 'lane_publisher_'):
                return
            tmsg = self._waypoints_to_trajectory(lane_points)
            self.lane_publisher_.publish(tmsg)
            self.get_logger().debug(f'Published igvc_lane with {len(tmsg.waypoints)} points')
        except Exception as e:
            self.get_logger().debug(f'Failed to publish lane centerline: {e}')
