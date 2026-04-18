import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry, Path
from std_msgs.msg import Header, String
from cev_msgs.msg import Waypoint, Trajectory
try:
    from cev_msgs.srv import QueryCostmap
except ImportError:
    QueryCostmap = None
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

        # Subscribe to planner mode so the sim can display it
        self.planner_mode = 'GPS'  # default
        self.planner_waypoint_idx = -1
        try:
            self.mode_sub = self.create_subscription(String, 'planner_mode', self._mode_callback, 10)
        except Exception:
            self.get_logger().warning('Could not create planner_mode subscriber')

        # Subscribe to planner's detected lane centerline for visualization
        self.detected_lane_pts = []  # list of (x, y) — latest frame only
        self.cumulative_lane_trail = []  # list of (x, y) — accumulated over entire run
        self._trail_dedup_dist = 0.15  # min distance between consecutive trail points
        try:
            self.lane_cl_sub = self.create_subscription(Path, 'detected_lane_cl', self._lane_cl_callback, 10)
        except Exception:
            self.get_logger().warning('Could not create detected_lane_cl subscriber')

        # Lane boundary publishers (simulate camera-based lane segmentation output).
        # Use TRANSIENT_LOCAL so late-connecting subscribers (planner) get the last message
        # immediately on connection, avoiding the startup race condition.
        self.last_lane_left_pts = []   # list of (x,y) — last published left boundary
        self.last_lane_right_pts = []  # list of (x,y) — last published right boundary
        self.last_lane_wall_left_pts = []   # dense visible left wall points for occupancy
        self.last_lane_wall_right_pts = []  # dense visible right wall points for occupancy
        # Persistent lane-wall memory: once seen, keep the wall point available even
        # after it leaves the current camera frustum.
        self._acc_lane_left_set: set = set()
        self._acc_lane_right_set: set = set()
        self.acc_lane_left_pts: List[Tuple[float,float]] = []
        self.acc_lane_right_pts: List[Tuple[float,float]] = []
        # Last raw occupancy grid — for debug heatmap overlay.
        self.last_grid_arr    = None
        self.last_grid_origin = (0.0, 0.0)
        self.last_grid_res    = 0.1
        # Planner’s Gaussian cost field [0,100] — received from /cost_field topic.
        self.last_cost_arr    = None
        try:
            self.cost_field_sub = self.create_subscription(
                OccupancyGrid, 'cost_field', self._cost_field_callback, 10)
        except Exception:
            self.cost_field_sub = None
        try:
            lane_qos = QoSProfile(depth=1)
            lane_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            lane_qos.reliability = QoSReliabilityPolicy.RELIABLE
            self.lane_left_pub_  = self.create_publisher(Path, 'lane/left',  lane_qos)
            self.lane_right_pub_ = self.create_publisher(Path, 'lane/right', lane_qos)
            self.lane_cl_pub_    = self.create_publisher(Path, 'lane/centerline', lane_qos)
        except Exception:
            self.lane_left_pub_  = None
            self.lane_right_pub_ = None
            self.lane_cl_pub_    = None
            self.get_logger().warning('Could not create lane boundary publishers')

        # Subscribe to planner's MPC trajectory for drawing target waypoints
        self.mpc_target_pts = []  # list of (x, y) — the waypoints fed to MPC this frame
        try:
            self.local_path_sub = self.create_subscription(Path, 'local_path', self._local_path_callback, 10)
        except Exception:
            self.get_logger().warning('Could not create local_path subscriber')

        # Service client used for planner costmap queries in debug tools
        self._costmap_query_service = '/query_costmap'
        self._costmap_query_timeout = 0.25
        self._costmap_client = None
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

    # ------------------------------------------------------------------
    # Local-window occupancy grid rasterizer.
    # Simulates LiDAR: only rasterizes obstacles that overlap the window.
    # No precompute — each call builds exactly the local area from scratch.
    # At 12m×12m / 0.1m = 120×120 = 14,400 cells with AABB pre-filter this
    # runs in <5 ms even with 30+ obstacles.
    # ------------------------------------------------------------------
    _GRID_RES = 0.1  # 10 cm resolution

    def _publish_occ_msg(self, arr, nx, ny, origin_x, origin_y, frame_id):
        """Publish a numpy int8 array (shape nx×ny, indexed [ix,iy]) as OccupancyGrid."""
        occ = OccupancyGrid()
        occ.header = Header()
        occ.header.stamp = self.get_clock().now().to_msg()
        occ.header.frame_id = frame_id
        info = MapMetaData()
        info.map_load_time = occ.header.stamp
        info.resolution = float(self._GRID_RES)
        info.width = nx
        info.height = ny
        info.origin.position.x = float(origin_x)
        info.origin.position.y = float(origin_y)
        info.origin.orientation.w = 1.0
        occ.info = info
        # OccupancyGrid data is row-major: data[iy*nx + ix]
        # arr[ix, iy] with Fortran-order flatten gives exactly that layout.
        occ.data = arr.flatten(order='F').tolist()
        try: self.occ_publisher_.publish(occ)
        except Exception: pass
        try: self.map_publisher_.publish(occ)
        except Exception: pass

    def publish_occupancy_grid(self, obstacles, width_m=12.0, height_m=12.0, resolution=None, frame_id='map'):
        """Publish a grid centred at world origin."""
        self.publish_occupancy_grid_local(obstacles, 0.0, 0.0,
                                          width_m=width_m, height_m=height_m, frame_id=frame_id)

    def publish_occupancy_grid_local(self, obstacles, center_x, center_y,
                                     width_m=12.0, height_m=12.0, resolution=None, frame_id='map'):
        """Rasterize visible obstacles into a local window centred on (center_x, center_y).
        Simulates incremental sensor data — no global map, just what's nearby."""
        res = self._GRID_RES
        nx = int(round(width_m  / res))
        ny = int(round(height_m / res))
        ox = center_x - width_m  / 2.0
        oy = center_y - height_m / 2.0
        ex = ox + width_m
        ey = oy + height_m

        # Cell-centre coordinates: arr[ix, iy] = world point
        xs = (np.arange(nx) + 0.5) * res + ox  # shape (nx,)
        ys = (np.arange(ny) + 0.5) * res + oy  # shape (ny,)
        gx, gy = np.meshgrid(xs, ys, indexing='ij')  # (nx, ny)

        arr = np.zeros((nx, ny), dtype=np.int8)

        for obs in obstacles:
            # Skip white lane-marking polygons — they are camera-detected (lane
            # boundary topics), not lidar-detected.  They should NOT appear in
            # the occupancy grid.
            if hasattr(obs, 'color'):
                c = obs.color
                if c[0] > 200 and c[1] > 200 and c[2] > 200:
                    continue
            if hasattr(obs, 'radius'):
                # AABB filter
                if obs.x + obs.radius < ox or obs.x - obs.radius > ex: continue
                if obs.y + obs.radius < oy or obs.y - obs.radius > ey: continue
                d2 = (gx - obs.x) ** 2 + (gy - obs.y) ** 2
                arr[d2 <= obs.radius ** 2] = 100
            elif hasattr(obs, 'vertices') and len(obs.vertices) >= 3:
                verts = obs.vertices
                vxs = [v[0] for v in verts]; vys = [v[1] for v in verts]
                # AABB filter
                if max(vxs) < ox or min(vxs) > ex: continue
                if max(vys) < oy or min(vys) > ey: continue
                # Numpy ray-casting point-in-polygon
                inside = np.zeros((nx, ny), dtype=bool)
                nv = len(verts); j = nv - 1
                for i in range(nv):
                    xi, yi = verts[i]; xj, yj = verts[j]
                    cond = ((gy > yi) != (gy > yj)) & \
                           (gx < (xj - xi) * (gy - yi) / (yj - yi + 1e-12) + xi)
                    inside ^= cond
                    j = i
                arr[inside] = 100

        # Add lane boundaries as hard obstacles.
        # Current visible walls are connected into thick segments so the planner
        # cannot slip through gaps. Persisted wall points keep already-seen
        # boundaries active even after they leave the detection frustum.
        lane_wall_radius = 0.45
        max_link_dist = 0.55

        def _paint_disc(px, py, radius):
            if px + radius < ox or px - radius > ex:
                return
            if py + radius < oy or py - radius > ey:
                return
            d2 = (gx - px) ** 2 + (gy - py) ** 2
            arr[d2 <= radius * radius] = 100

        def _raster_lane_wall(points):
            if not points:
                return

            prev = None
            for px, py in points:
                _paint_disc(px, py, lane_wall_radius)
                if prev is not None:
                    seg_len = math.hypot(px - prev[0], py - prev[1])
                    if seg_len <= max_link_dist:
                        steps = max(1, int(math.ceil(seg_len / (lane_wall_radius * 0.6))))
                        for step in range(1, steps):
                            t = step / steps
                            sx = prev[0] + t * (px - prev[0])
                            sy = prev[1] + t * (py - prev[1])
                            _paint_disc(sx, sy, lane_wall_radius)
                prev = (px, py)

        _raster_lane_wall(list(getattr(self, 'last_lane_wall_left_pts', [])))
        _raster_lane_wall(list(getattr(self, 'last_lane_wall_right_pts', [])))

        self._publish_occ_msg(arr, nx, ny, ox, oy, frame_id)
        # Store raw grid for debug heatmap (Gaussian blur computed in draw.py).
        self.last_grid_arr    = arr
        self.last_grid_origin = (ox, oy)
        self.last_grid_res    = res

    def publish_lane_boundaries(self, obstacles, car_x: float, car_y: float,
                                car_heading: float) -> None:
        """Simulate a forward-facing camera detecting white lane markings.

        Detection frustum: triangular, directly in front of the car, 0-4 m forward.
        A detected point goes to lane/left (blue) if it is to the LEFT of the car
        centreline, lane/right (red) if to the RIGHT.
        Points outside the frustum are simply not detected.
        """
        if self.lane_left_pub_ is None or self.lane_right_pub_ is None:
            return

        cos_h = math.cos(car_heading)
        sin_h = math.sin(car_heading)
        DETECT_FWD = 4.0   # forward detection range (m)
        DETECT_LAT = math.tan(math.radians(75.0)) * DETECT_FWD  # 150° tip angle
        FWD_BIN = 0.35  # keep the nearest visible boundary per ~35cm forward slice

        left_pts:  List[Tuple[float, float, float, float]] = []  # (lat, fwd, x, y)
        right_pts: List[Tuple[float, float, float, float]] = []
        dense_left_wall_pts: List[Tuple[float, float, float, float]] = []
        dense_right_wall_pts: List[Tuple[float, float, float, float]] = []

        for obs in obstacles:
            if not (hasattr(obs, 'vertices') and hasattr(obs, 'color')):
                continue
            c = obs.color
            if not (c[0] > 200 and c[1] > 200 and c[2] > 200):
                continue
            v = obs.vertices
            if len(v) < 4:
                continue

            spine = self._polygon_spine_points(v, samples=6)
            for (sx, sy) in spine:
                dx = sx - car_x
                dy = sy - car_y
                fwd = cos_h * dx + sin_h * dy
                lat = -sin_h * dx + cos_h * dy
                if fwd < 0.0 or fwd > DETECT_FWD:
                    continue
                # Triangular frustum: width expands linearly with forward distance.
                lat_limit = DETECT_LAT * (fwd / DETECT_FWD)
                if abs(lat) > lat_limit:
                    continue
                if lat >= 0.0:
                    left_pts.append((lat, fwd, sx, sy))
                    dense_left_wall_pts.append((lat, fwd, sx, sy))
                else:
                    right_pts.append((lat, fwd, sx, sy))
                    dense_right_wall_pts.append((lat, fwd, sx, sy))

        # Keep the nearest visible boundary in each forward slice instead of using a
        # single global nearest-side filter. The global filter breaks on bends: one side
        # can survive only in near slices while the other survives only in far slices,
        # leaving no overlapping forward range for centerline interpolation.
        def _nearest_per_fwd_bin(pts, keep_left: bool):
            if not pts:
                return pts
            best_by_bin = {}
            for lat, fwd, x, y in pts:
                key = int(round(fwd / FWD_BIN))
                prev = best_by_bin.get(key)
                if prev is None:
                    best_by_bin[key] = (lat, fwd, x, y)
                    continue
                if keep_left:
                    if lat < prev[0]:
                        best_by_bin[key] = (lat, fwd, x, y)
                else:
                    if lat > prev[0]:
                        best_by_bin[key] = (lat, fwd, x, y)
            kept = list(best_by_bin.values())
            kept.sort(key=lambda p: p[1])
            return kept

        def _dense_wall_points(pts):
            pts = sorted(pts, key=lambda p: (p[1], abs(p[0])))
            out = []
            seen = set()
            for _, _, x, y in pts:
                key = (round(x * 20), round(y * 20))
                if key in seen:
                    continue
                seen.add(key)
                out.append((x, y))
            return out

        self.last_lane_wall_left_pts = _dense_wall_points(dense_left_wall_pts)
        self.last_lane_wall_right_pts = _dense_wall_points(dense_right_wall_pts)

        def _accumulate_unique(target_pts, target_set, new_pts):
            for x, y in new_pts:
                key = (round(x * 20), round(y * 20))
                if key in target_set:
                    continue
                target_set.add(key)
                target_pts.append((x, y))

        _accumulate_unique(self.acc_lane_left_pts, self._acc_lane_left_set,
                           self.last_lane_wall_left_pts)
        _accumulate_unique(self.acc_lane_right_pts, self._acc_lane_right_set,
                           self.last_lane_wall_right_pts)

        def _persistent_candidates(memory_pts, is_left: bool):
            kept = []
            for x, y in memory_pts:
                dx = x - car_x
                dy = y - car_y
                fwd = cos_h * dx + sin_h * dy
                lat = -sin_h * dx + cos_h * dy
                if fwd < -0.5 or fwd > 6.0:
                    continue
                if is_left:
                    if lat < 0.05 or lat > 3.5:
                        continue
                else:
                    if lat > -0.05 or lat < -3.5:
                        continue
                kept.append((lat, fwd, x, y))
            return kept

        left_pts = _nearest_per_fwd_bin(left_pts + _persistent_candidates(self.acc_lane_left_pts, True),
                                        keep_left=True)
        right_pts = _nearest_per_fwd_bin(right_pts + _persistent_candidates(self.acc_lane_right_pts, False),
                                         keep_left=False)

        left_pts.sort(key=lambda p: p[1])   # sort by fwd
        right_pts.sort(key=lambda p: p[1])

        from geometry_msgs.msg import PoseStamped as _PS
        now = self.get_clock().now().to_msg()

        def _make_path(pts):
            msg = Path()
            msg.header.stamp = now
            msg.header.frame_id = 'map'
            for _, _, x, y in pts:
                ps = _PS()
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(y)
                msg.poses.append(ps)
            return msg

        self.lane_left_pub_.publish(_make_path(left_pts))
        self.lane_right_pub_.publish(_make_path(right_pts))
        if not hasattr(self, '_lane_pub_count'):
            self._lane_pub_count = 0
        self._lane_pub_count += 1
        if self._lane_pub_count <= 3 or self._lane_pub_count % 50 == 0:
            # Debug: show obstacle colors we're scanning
            white_count = 0
            for obs in obstacles:
                if hasattr(obs, 'vertices') and hasattr(obs, 'color'):
                    c = obs.color
                    if c[0] > 200 and c[1] > 200 and c[2] > 200:
                        white_count += 1
            total = len(obstacles)
            print(f"[LANE PUB #{self._lane_pub_count}] left={len(left_pts)} right={len(right_pts)} obstacles={total} white={white_count}")
        # Store for sim visualization
        self.last_lane_left_pts = [(x, y) for _, _, x, y in left_pts]
        self.last_lane_right_pts = [(x, y) for _, _, x, y in right_pts]
    def publish_persistent_centerline(self, center_pts: List[Tuple[float, float]]) -> None:
        """Publish the accumulated lane centerline as a Path on lane/centerline.
        Uses TRANSIENT_LOCAL so the planner always gets the latest full history."""
        if self.lane_cl_pub_ is None:
            return
        from geometry_msgs.msg import PoseStamped as _PS
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in center_pts:
            ps = _PS()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            msg.poses.append(ps)
        self.lane_cl_pub_.publish(msg)

    @staticmethod
    def _polygon_spine_points(vertices, samples: int = 6) -> List[Tuple[float, float]]:
        """Return points sampled along the medial axis of a thin 4-vertex polygon strip.
        Adaptive: ~0.3m spacing, minimum 3 samples."""
        v = vertices
        def _mid(a, b):
            return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        def _dist(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])
        # Candidate spine: midpoints of opposite edge pairs
        m01_23 = (_mid(v[0], v[1]), _mid(v[2], v[3]))
        m12_30 = (_mid(v[1], v[2]), _mid(v[3], v[0]))
        if _dist(*m01_23) >= _dist(*m12_30):
            s0, s1 = m01_23
        else:
            s0, s1 = m12_30
        # Adaptive sampling: ~0.3m intervals, min 3
        length = _dist(s0, s1)
        n_samples = max(3, int(length / 0.3) + 1)
        pts = []
        for k in range(n_samples + 1):
            t = k / n_samples
            pts.append((s0[0] + t * (s1[0] - s0[0]), s0[1] + t * (s1[1] - s0[1])))
        return pts

    def _trajectory_callback(self, msg: Trajectory):
        # store last received trajectory message for simulator to consume
        self.latest_trajectory_msg = msg
        self.get_logger().info('Received trajectory from planner')

    def _mode_callback(self, msg: String):
        # Format: "LANE" or "GPS:N" where N is the waypoint index being targeted
        raw = msg.data
        parts = raw.split(':')
        self.planner_mode = parts[0]               # 'LANE' or 'GPS'
        self.planner_waypoint_idx = int(parts[1]) if len(parts) > 1 else -1

    def _lane_cl_callback(self, msg: Path):
        self.detected_lane_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        # Accumulate into trail (deduplicate by distance)
        for pt in self.detected_lane_pts:
            if not self.cumulative_lane_trail:
                self.cumulative_lane_trail.append(pt)
            else:
                lx, ly = self.cumulative_lane_trail[-1]
                dx, dy = pt[0] - lx, pt[1] - ly
                if dx * dx + dy * dy >= self._trail_dedup_dist ** 2:
                    self.cumulative_lane_trail.append(pt)

    def _cost_field_callback(self, msg: OccupancyGrid):
        """Receive the planner's Gaussian cost field for debug heatmap."""
        w, h = msg.info.width, msg.info.height
        raw = np.array(msg.data, dtype=np.float32)
        # OccupancyGrid is row-major: data[y*width + x], but our grid uses (x,y)
        # indexing so reshape to (width, height) by reshaping (height, width) then transposing.
        self.last_cost_arr = raw.reshape(h, w).T
        self.last_grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.last_grid_res    = msg.info.resolution

    def _local_path_callback(self, msg: Path):
        self.mpc_target_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

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
