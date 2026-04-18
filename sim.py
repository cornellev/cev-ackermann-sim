import pygame
import math
import os
import json
import numpy as np
import rclpy
import threading
from objects import CircleObstacle, CollisionDetector, PolygonObstacle, LineObstacle
from sim_map_loader import load_map_file, load_obstacles_from_json, load_lane_objects_from_json
from constants import *
from sim_publisher import VehiclePublisher
from sim_edit import SceneEditor
from draw import *
<<<<<<< HEAD
from vehicle import Vehicle, calculate_angular_velocity
from evaluator import RunEvaluator
import numpy as np
pygame.init()

=======
from nearest_costmap import Grid as CostGrid, NearestGenerator
from euclidean_costmap import Grid as CostGrid, Euclidean, EuclideanCostMap
pygame.init()

def calculate_angular_velocity(speed, steering_angle, wheelbase):
    if abs(steering_angle) > 1e-6:
        turning_radius = wheelbase / math.tan(steering_angle)
        return speed / turning_radius
    else:
        return 0

class Vehicle:
    def __init__(self, x=0, y=0):
        # Vehicle parameters in meters
        self.wheelbase = 0.4572  # L (18 inches)
        self.track_width = 0.3048  # W (12 inches)
        self.length = 0.9144 # 3 feet in meters
        self.width = 0.6096  # 2 feet in meters

        self.max_speed = 2.2352  # m/s (5 mph)
        self.throttle_acceleration = 2.5  # m/s^2
        self.max_acceleration = 2.5  # m/s^2
        self.steering_rate = math.radians(180) # rad/s
        self.max_steering_rate = math.radians(180) # rad/s

        # TODO: steering system parameters
        # parameters go here

        # State variables
        self.x = x
        self.y = y
        self.heading = 0.0  # radians
        self.speed = 0.0  # m/s
        # assuming bicycle model with 100% ackermann
        self.steering_angle = 0.0  # radians

        self.max_steering_angle = self.calculate_max_steering_angle()

    def force_update(self, dt, speed, steering_angle):
        """Updates vehicle state with given parameters from trajectory follower. 
        If the given speed exceeds the max velocity, clamp down to max velocity
        If the given steering angle exceeds max steering angle speed, clamp down as well
        """
        if abs(self.speed - speed) > self.max_acceleration * dt:
            speed = self.speed + self.max_acceleration * dt if speed > self.speed else self.speed - self.max_acceleration * dt
        if abs(speed) > self.max_speed:
            speed = self.max_speed if speed > 0 else -self.max_speed
        
        if abs(self.steering_angle - steering_angle) > self.max_steering_rate * dt:
            steering_angle = self.steering_angle + self.max_steering_rate * dt if steering_angle > self.steering_angle else self.steering_angle - self.max_steering_rate * dt
        if abs(steering_angle) > self.max_steering_angle:
            steering_angle = self.max_steering_angle if steering_angle > 0 else -self.max_steering_angle

        self.speed = speed
        self.steering_angle = steering_angle

        angular_velocity = calculate_angular_velocity(self.speed, self.steering_angle, self.wheelbase)
        self.heading += angular_velocity * dt
        # Normalize heading to be within -pi to pi
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi

        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

    def update(self, dt, speed_input, steer_input):
        """Update vehicle state w/ bicycle model"""

        # update speed based on throttle input
        self.speed = max(-self.max_speed, min(self.max_speed, self.speed + speed_input * self.throttle_acceleration * dt))
        if speed_input < 1e-6:
            # natural deceleration
            self.speed *= 0.25 ** dt

        # update effective steering angle based on steering input
        if steer_input != 0:
            self.steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, self.steering_angle + steer_input * self.steering_rate * dt))
        else:
            self.steering_angle = self.steering_angle * 0.9 # natural return to center
        # update position and heading
        angular_velocity = calculate_angular_velocity(self.speed, self.steering_angle, self.wheelbase)

        # Calculate heading
        self.heading += angular_velocity * dt
        # Normalize heading to be within -pi to pi
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi

        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

    def get_corners(self):
        """Returns the world coordinates of the four corners of the vehicle."""
        half_length = self.length / 2
        half_width = self.width / 2

        # Corners in local vehicle frame (front-left, front-right, back-right, back-left)
        local_corners = [
            (half_length, half_width),
            (half_length, -half_width),
            (-half_length, -half_width),
            (-half_length, half_width)
        ]

        # Rotate and translate corners to world frame
        world_corners = []
        for x_local, y_local in local_corners:
            x_world = self.x + x_local * math.cos(self.heading) - y_local * math.sin(self.heading)
            y_world = self.y + x_local * math.sin(self.heading) + y_local * math.cos(self.heading)
            world_corners.append((x_world, y_world))

        return world_corners
    def calculate_max_steering_angle(self):
        """
        Calculate maximum steering angle from rack & pinion geometry.
        This is the maximum angle a wheel can turn, NOT the maximum effective steering angle
        (though the two should be close)
        """
        # arccot of average of cot of outer wheels 
        return math.radians(27.2) # 34.919 deg, 23.0319 deg

>>>>>>> refs/remotes/origin/lane-planner
class Simulator:
    def __init__(self, scene_arg: str = None):
        import time
        t0 = time.time()
        print("[STARTUP] Starting simulator initialization...")

        self._shutdown_complete = False
        self._owns_rclpy_context = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True

        t1 = time.time()
        pygame.display.set_caption("Vehicle Simulator")
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 18)
        t2 = time.time()
        print(f"[STARTUP] Pygame display setup took {(t2-t1)*1000:.1f}ms")
        
        # UI state for map selector
        self.maps_dir = os.path.join(os.path.dirname(__file__), 'maps')
        self.available_maps = self.scan_maps_directory()
        self.current_map = None
        self.dropdown_open = False
        self.edit_button_rect = pygame.Rect(SCREEN_WIDTH - 60, 10, 50, 30)
        self.distance_button_rect = pygame.Rect(self.edit_button_rect.x - 10 - 70, 10, 70, 30)
        self.dropdown_rect = pygame.Rect(self.distance_button_rect.x - 10 - 120, 10, 120, 30)
        
        # Editor state
        self.edit_mode = False
        self.editor = None  # Will be initialized when entering edit mode
        self.show_map_controls = True  # Toggle for dropdown/edit visibility
        
        # Initialize publisher before loading maps
        self.pose_publisher = VehiclePublisher()
        t3 = time.time()
        print(f"[STARTUP] VehiclePublisher created in {(t3-t2)*1000:.1f}ms")
        
        # load scene from config if available (obstacles, start, goal)
        self.vehicle = Vehicle()
        self.vehicle.heading = 0.0
        self.vehicle.speed = 0.0
        t4 = time.time()
        print(f"[STARTUP] Vehicle created in {(t4-t3)*1000:.1f}ms")

        # Local plan cost debug instrumentation (toggle via show_traj_cost_debug)
        self.show_traj_cost_debug = False
        self._traj_cost_rows = []
        self._traj_cost_timestep = None
        self._last_traj_stamp = None
        self._costmap_width = 30.0
        self._costmap_height = 30.0
        self._costmap_resolution = 0.1
        self._costmap_origin = (-self._costmap_width / 2.0, -self._costmap_height / 2.0)
        self._costmap_search_radius = 3.5
        self._costmap_kernel_sigma = 1
        self._costmap_generator = None
        self._costmap = None
        self._costmap_shape = (0, 0)
        self._last_ros_costmap_seq = -1
        self._target_debug_velocity = self.vehicle.max_speed * (2.0 / 3.0)
        self._costmap_interp_resolution = 0.1
        self._cost_weights = {
            # Defaults mirror LaneFollowingMPC::costs
            "along_track": 200.0,
            "cte": 50.0,
            "costmap": 50.0,
            "obs_threshold": 5.0,
            "cte_threshold": 1.5,
            "along_threshold": 0.0,
        }
        self._costmap_probe = None
        self._costmap_probe_lifetime = 4.0
        self._distance_points = []
        self._distance_value = None
        self.distance_measure_enabled = False
        self._refresh_costmap_generator()
        self._show_planner_ghosts = False
        # UI + ROS publish state for live cost tuning
        self._cost_controls = []
        self._cost_control_layout = {}
        self._cost_panel_rect = None
        self._active_slider_key = None
        self._slider_dragging = False
        self._active_input_key = None
        self._input_buffer = ""
        self._input_caret = 0
        self._last_published_cost_weights = None
        self._init_cost_tuning_ui()

        # Load initial map
        if scene_arg and os.path.exists(scene_arg):
            self.load_map(scene_arg)
        elif len(self.available_maps) > 0:
            self.load_map(os.path.join(self.maps_dir, self.available_maps[0]))
        else:
            self.load_empty_map()
        t5 = time.time()
        print(f"[STARTUP] Map loading took {(t5-t4)*1000:.1f}ms")

        # collision detector used each frame
        self.collision_detector = CollisionDetector()
        self.is_colliding = False

        self.camera_x = 0
        self.camera_y = 0


        self.active_trajectory = None
        self.traj_index = 0
        self.follow_planner = False

        self.use_pure_pursuit = False

        # Current waypoint index for sequential MPC target publishing
        self._waypoint_index = 0
        self._waypoint_reach_dist = 1.0  # meters to consider waypoint reached

        # Camera zoom control
        self.camera_zoom = 1.0  # Zoom factor (1.0 = normal, 0.5 = zoomed out 2x, 2.0 = zoomed in 2x)
        self.camera_zoom_min = 0.25
        self.camera_zoom_max = 3.0

        # Fog-of-war perception mode: only draw obstacles the car has "seen"
        self.perception_mode = False          # V key toggles
        self.perception_range = 5.0           # metres — matches lane boundary max_dist
        self._revealed_obstacle_ids = set()   # id(obs) of obstacles seen at least once
        self._permanent_lane_left = []        # (x,y) accumulated from all frames
        self._permanent_lane_right = []       # (x,y) accumulated from all frames
        self._permanent_lane_center = []      # (x,y) midpoints, only when both sides seen

        # Debug mode: D key toggles heatmap overlay + click-to-query cost
        self.debug_mode = False
        self._debug_click_pos = None          # last clicked (world_x, world_y) for label

        # Local occupancy grid generation (lazy loading)
        self._last_grid_update_pos = (0.0, 0.0)  # Track last grid computation position
        self._grid_update_threshold = 1.5  # Re-publish window when vehicle moves > 1.5m
        self._last_grid_update_time = pygame.time.get_ticks() / 1000.0
        
        # Asynchronous grid computation (non-blocking background thread)
        self._grid_thread = None
        self._grid_thread_lock = threading.Lock()
        self._grid_should_compute = False
        self._grid_compute_params = None

        # NOTE: Initial full-map occupancy grid generation is now skipped for fast startup.
        # The lazy local grid generator (publish_occupancy_grid_local) will compute
        # the initial grid around the vehicle on the first frame of the main loop.
        # This provides instant simulator startup for even complex maps.
        # try:
        #     self.pose_publisher.publish_occupancy_grid(self.obstacles, width_m=30.0, height_m=30.0, resolution=0.1)
        # except Exception as e:
        #     print(f"Failed to publish initial occupancy grid: {e}")

        try:
            # publish stuff
            self.pose_publisher.publish_pose(self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.vehicle.speed, self.vehicle.steering_angle)
            self.pose_publisher.publish_odometry(self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.vehicle.speed)
        except Exception:
            pass

        # Publish initial occupancy grid centred on the vehicle's start position.
        # (The lazy loop only re-publishes when the car has moved 1.5m so we need this upfront.)
        try:
            self.pose_publisher.publish_lane_boundaries(
                self.obstacles,
                car_x=self.vehicle.x,
                car_y=self.vehicle.y,
                car_heading=self.vehicle.heading,
            )
            self.pose_publisher.publish_occupancy_grid_local(
                self.obstacles,
                center_x=self.vehicle.x,
                center_y=self.vehicle.y,
            )
        except Exception:
            pass

        # start/goal should already be set by load_map() or load_empty_map()
        # Ensure attributes exist and provide sensible defaults if not.
        if not hasattr(self, 'start_pose'):
            self.start_pose = (float(self.vehicle.x), float(self.vehicle.y), float(self.vehicle.heading))
        if not hasattr(self, 'target_pose'):
            self.target_pose = (12.0, 5.0, 0.0)

        self.publish_interval = 0.1
        self._last_publish_time = pygame.time.get_ticks() / 1000.0
        
        t_end = time.time()
        print(f"[STARTUP] Total initialization took {(t_end-t0)*1000:.1f}ms - READY TO RENDER")
        print("[CONTROLS] SPACE=toggle planner | WASD=manual drive | V=fog-of-war mode | +/-=zoom")
        print("[STARTUP] Note: Occupancy grid is computed lazily in the main loop (15-20x faster)")
        print("[STARTUP]       - Never blocks initial startup (<120ms even for complex maps)")
        print("[STARTUP]       - Updates as vehicle moves (every 0.5s+ with 900-950ms per update)")
        print("[STARTUP]       - Uses local 30m×30m window at 0.2m resolution for speed")

    def _shutdown_sim(self):
        if self._shutdown_complete:
            return
        self._shutdown_complete = True

        pygame.quit()
        try:
            self.pose_publisher.destroy_node()
        except Exception:
            pass
        if self._owns_rclpy_context and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def handle_input(self):
        """Interpret user input"""
        keys = pygame.key.get_pressed()

        # ternary throttle (forward, backward, no throttle)
        speed_input = 0
        if keys[pygame.K_w]:
            speed_input = 1
        elif keys[pygame.K_s]:
            speed_input = -1
        else:
            speed_input = 0

        # move steering wheel left, right, or keep it in place
        steer_input = 0
        if keys[pygame.K_a]:
            steer_input = 1
        elif keys[pygame.K_d]:
            steer_input = -1
        else:
            steer_input = 0

        # Zoom controls
        if keys[pygame.K_EQUALS] or keys[pygame.K_PLUS]:
            if self.edit_mode and self.editor:
                self.editor.camera_zoom = min(self.editor.camera_zoom_max, self.editor.camera_zoom * 1.02)
            else:
                self.camera_zoom = min(self.camera_zoom_max, self.camera_zoom * 1.02)
        if keys[pygame.K_MINUS]:
            if self.edit_mode and self.editor:
                self.editor.camera_zoom = max(self.editor.camera_zoom_min, self.editor.camera_zoom * 0.98)
            else:
                self.camera_zoom = max(self.camera_zoom_min, self.camera_zoom * 0.98)
        if keys[pygame.K_0]:
            if self.edit_mode and self.editor:
                self.editor.camera_zoom = 1.0  # Reset to normal zoom
            else:
                self.camera_zoom = 1.0  # Reset to normal zoom

        # Pure Pursuit toggle (U key)
        u_pressed = keys[pygame.K_u]
        if u_pressed and not getattr(self, '_u_was_pressed', False):
            self.use_pure_pursuit = not self.use_pure_pursuit
            print(f"Pure Pursuit: {'ON' if self.use_pure_pursuit else 'OFF'}")
        self._u_was_pressed = u_pressed

        # TODO: add keys to change car dimensions (or have them be input at start of program?)

        return speed_input, steer_input

    @staticmethod
    def _point_in_circle(x, y, cx, cy, r):
        dx = x - cx
        dy = y - cy
        return dx * dx + dy * dy <= r * r

    @staticmethod
    def _point_in_polygon(x, y, polygon):
        if not polygon:
            return False
        inside = False
        j = len(polygon) - 1
        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            intersects = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            )
            if intersects:
                inside = not inside
            j = i
        return inside

    def _point_inside_obstacles(self, x, y):
        for obs in self.obstacles:
            if isinstance(obs, CircleObstacle):
                if self._point_in_circle(x, y, obs.x, obs.y, obs.radius):
                    return True
            elif hasattr(obs, 'vertices'):
                if self._point_in_polygon(x, y, obs.vertices):
                    return True
        return False

<<<<<<< HEAD
    def _generate_occupancy_grid(self, center_x=None, center_y=None, width_m=30.0, height_m=30.0, resolution=0.2):
        """
        Generate local occupancy grid around vehicle position.
        
        Args:
            center_x, center_y: Center of grid (defaults to vehicle position)
            width_m, height_m: Grid size in meters
            resolution: Grid cell size in meters
        
        Returns:
            numpy array with 0=free, 1=occupied, shape=(height_cells, width_cells)
        """
        if center_x is None:
            center_x = self.vehicle.x
        if center_y is None:
            center_y = self.vehicle.y
        
        nx = int(math.ceil(width_m / resolution))
        ny = int(math.ceil(height_m / resolution))
        
        # Origin at bottom-left
        origin_x = center_x - width_m / 2.0
        origin_y = center_y - height_m / 2.0
        
        # Create occupancy grid (0=free, 1=occupied)
        grid = np.zeros((ny, nx), dtype=np.float32)
        
        for iy in range(ny):
            for ix in range(nx):
                # Cell center in world coordinates
                cx = origin_x + (ix + 0.5) * resolution
                cy = origin_y + (iy + 0.5) * resolution
                
                # Check if occupied
                if self._point_inside_obstacles(cx, cy):
                    grid[iy, ix] = 1.0
        
        return grid

    def poll_lidar_rays(self, num_rays: int = 36, max_distance: float = 10.0) -> list:
        """
        Poll LiDAR rays directly from obstacles (vehicle-relative).
        Much faster than generating full occupancy grid.
        
        Args:
            num_rays: Number of rays to cast (36 = 10° resolution)
            max_distance: Maximum ray distance in meters
        
        Returns:
            List of distances for each ray angle
        """
        distances = []
        
        for ray_idx in range(num_rays):
            # Ray angle relative to vehicle heading
            angle_relative = (ray_idx / num_rays) * 2 * math.pi
            angle_world = self.vehicle.heading + angle_relative
            
            # Ray cast from vehicle position outward
            ray_distance = max_distance
            
            # Check distance to each obstacle along this ray
            step_size = 0.1  # 10cm per check
            for check_distance in np.arange(0, max_distance, step_size):
                ray_x = self.vehicle.x + check_distance * math.cos(angle_world)
                ray_y = self.vehicle.y + check_distance * math.sin(angle_world)
                
                if self._point_inside_obstacles(ray_x, ray_y):
                    ray_distance = check_distance
                    break
            
            distances.append(ray_distance)
        
        return distances
=======
    def _refresh_costmap_generator(self):
        self._costmap_generator = Euclidean()

    def _refresh_costmap_from_ros(self, force=False):
        publisher = getattr(self, 'pose_publisher', None)
        if publisher is None:
            return False
        try:
            seq = publisher.costmap_seq() if hasattr(publisher, 'costmap_seq') else getattr(publisher, '_costmap_seq', 0)
        except Exception:
            seq = 0
        if not force and seq == self._last_ros_costmap_seq:
            return False

        meta = getattr(publisher, 'latest_costmap_meta', None)
        data_msg = getattr(publisher, 'latest_costmap_data', None)
        if meta is None or data_msg is None:
            return False

        width = int(getattr(meta, 'width', 0))
        height = int(getattr(meta, 'height', 0))
        if width <= 0 or height <= 0:
            return False

        data = getattr(data_msg, 'data', None)
        if data is None or len(data) < width * height:
            return False

        # ROS data is row-major (x fastest). Transpose so grid.data[x, y] aligns with C++.
        flat = np.array(list(data)[: width * height], dtype=np.float32)
        grid_data = flat.reshape((height, width)).T
        origin = (float(meta.origin.position.x), float(meta.origin.position.y))
        resolution = float(getattr(meta, 'resolution', 0.0)) or 0.1

        grid = CostGrid(data=grid_data, origin=origin, resolution=resolution)
        self._costmap = EuclideanCostMap(grid)
        self._costmap_origin = grid.origin
        self._costmap_shape = grid.data.shape
        self._costmap_resolution = grid.resolution
        self._last_ros_costmap_seq = seq
        return True

    def _rebuild_costmap_from_obstacles(self):
        if not self.show_traj_cost_debug:
            return
        if self._refresh_costmap_from_ros(force=True):
            return
        self._costmap = None

    def _generate_costmap_grid(self):
        nx = int(max(1, math.ceil(self._costmap_width / self._costmap_resolution)))
        ny = int(max(1, math.ceil(self._costmap_height / self._costmap_resolution)))
        origin_x = -self._costmap_width / 2.0
        origin_y = -self._costmap_height / 2.0
        data = np.zeros((nx, ny), dtype=np.float32)
        for ix in range(nx):
            cx = origin_x + (ix + 0.5) * self._costmap_resolution
            for iy in range(ny):
                cy = origin_y + (iy + 0.5) * self._costmap_resolution
                occupied = self._point_inside_obstacles(cx, cy)
                data[ix, iy] = 1.0 if occupied else 0.0
        cost_grid = CostGrid(data=data, origin=(origin_x, origin_y), resolution=self._costmap_resolution)
        self._costmap_origin = cost_grid.origin
        self._costmap_shape = cost_grid.data.shape
        return cost_grid
>>>>>>> refs/remotes/origin/lane-planner

    def _lookup_costmap_cost(self, x, y):
        if not self.show_traj_cost_debug:
            return (None, 'debug_disabled')
        if not self._costmap:
            return (None, 'no_data')
        grid = self._costmap.grid
        ix = int((x - grid.origin[0]) / grid.resolution)
        iy = int((y - grid.origin[1]) / grid.resolution)
        rows, cols = grid.data.shape
        if ix < 0 or iy < 0 or ix >= rows or iy >= cols:
            return (None, 'out_of_bounds')
        return (float( grid.data[ix, iy]), None)
        # return (float(0 if grid.data[ix, iy] > 5 else 100), None)

    def _interpolated_costmap_penalty(self, start, end=None, resolution=None):
        """Mirror LaneFollowingMPC::interpolated_costmap_penalty for debug costs."""
        if not self.show_traj_cost_debug or not self._costmap:
            return None
        resolution = resolution or self._costmap_interp_resolution
        if resolution <= 0.0:
            resolution = 0.1

        start_x, start_y = start
        if end is None:
            end_x, end_y = start_x, start_y
        else:
            end_x, end_y = end

        dx = end_x - start_x
        dy = end_y - start_y
        length = math.hypot(dx, dy)
        has_segment = end is not None and length > 1e-6
        steps = max(1, int(math.ceil(length / resolution))) if has_segment else 0

        def sample(t):
            px = start_x + t * dx
            py = start_y + t * dy
            cost_val, _ = self._lookup_costmap_cost(px, py)
            if cost_val is None or math.isinf(cost_val):
                return None
            return cost_val * (1.0 + 0.48 * (1.0 - cost_val))

        if steps == 0:
            return sample(0.0)

        penalty = 0.0
        for step in range(steps + 1):
            t = float(step) / float(steps)
            val = sample(t)
            if val is None:
                return None
            penalty += val
        return penalty

    def _record_costmap_probe(self, x, y, cost_val, reason=None):
        """Store the last probed costmap point for on-screen display."""
        self._costmap_probe = {
            'x': float(x),
            'y': float(y),
            'cost': None if cost_val is None else float(cost_val),
            'reason': reason,
            'timestamp': pygame.time.get_ticks() / 1000.0,
        }

    def _record_distance_point(self, x, y):
        if len(self._distance_points) >= 2:
            self._distance_points = []
            self._distance_value = None
        self._distance_points.append((float(x), float(y)))
        if len(self._distance_points) == 2:
            (x1, y1), (x2, y2) = self._distance_points
            self._distance_value = math.hypot(x2 - x1, y2 - y1)

    def _clear_distance_measurement(self):
        self._distance_points = []
        self._distance_value = None

    def _get_active_costmap_probe(self):
        if not self._costmap_probe or not self.show_traj_cost_debug:
            return None
        if self._costmap_probe_lifetime <= 0.0:
            return self._costmap_probe
        now = pygame.time.get_ticks() / 1000.0
        if now - self._costmap_probe['timestamp'] > self._costmap_probe_lifetime:
            self._costmap_probe = None
            return None
        return self._costmap_probe

    def _clear_costmap_probe(self):
        self._costmap_probe = None

    def _init_cost_tuning_ui(self):
        """Set up slider metadata for MPC lane cost tuning."""
        self._cost_controls = [
            {"label": "w_along", "key": "along_track", "min": 0.0, "max": 200.0, "step": 0.1, "default": self._cost_weights["along_track"]},
            {"label": "w_cte", "key": "cte", "min": 0.0, "max": 100.0, "step": 0.1, "default": self._cost_weights["cte"]},
            {"label": "w_costmap", "key": "costmap", "min": 0.0, "max": 100.0, "step": 0.1, "default": self._cost_weights["costmap"]},
            {"label": "obs_thr", "key": "obs_threshold", "min": 0.1, "max": 20.0, "step": 0.1, "default": self._cost_weights["obs_threshold"]},
            {"label": "cte_thr", "key": "cte_threshold", "min": 0.1, "max": 10.0, "step": 0.1, "default": self._cost_weights["cte_threshold"]},
            {"label": "along_thr", "key": "along_threshold", "min": 0.0, "max": 10.0, "step": 0.1, "default": self._cost_weights["along_threshold"]},
            {"label": "target_v", "key": "target_v", "min": 0.0, "max": self.vehicle.max_speed, "step": 0.05, "default": self._target_debug_velocity},
        ]
        for ctrl in self._cost_controls:
            ctrl["value"] = ctrl.get("default", 0.0)
        self._last_published_cost_weights = None
        self._maybe_publish_cost_weights(force=True)

    def _set_cost_control_value(self, key, value, clamp=True):
        # Clamp and assign to control + local dict
        clamped = value
        for ctrl in self._cost_controls:
            if ctrl["key"] == key:
                v_min, v_max = ctrl["min"], ctrl["max"]
                clamped = max(v_min, min(v_max, value)) if clamp else value
                ctrl["value"] = clamped
                break
        if key in self._cost_weights:
            self._cost_weights[key] = clamped
        elif key == "target_v":
            self._target_debug_velocity = clamped
        self._maybe_publish_cost_weights()

    def _maybe_publish_cost_weights(self, force=False):
        payload = (
            round(float(self._cost_weights.get("along_track", 0.0)), 4),
            round(float(self._cost_weights.get("cte", 0.0)), 4),
            round(float(self._cost_weights.get("costmap", 0.0)), 4),
            round(float(self._cost_weights.get("obs_threshold", 0.0)), 4),
            round(float(self._cost_weights.get("cte_threshold", 0.0)), 4),
            round(float(self._cost_weights.get("along_threshold", 0.0)), 4),
            round(float(self._target_debug_velocity), 4),
        )
        if force or payload != self._last_published_cost_weights:
            try:
                if getattr(self, 'pose_publisher', None) is not None:
                    self.pose_publisher.publish_lane_cost_weights(self._cost_weights, self._target_debug_velocity)
            except Exception:
                pass
            self._last_published_cost_weights = payload

    @staticmethod
    def _normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _compute_lane_errors(self, x, y, heading):
        lane = getattr(self, 'lane_centerline', None)
        if not lane or len(lane) < 2:
            return (None, None, None)
        best_dist = None
        best_heading = None
        best_sign = 0.0
        best_s = None
        arc_len = 0.0
        for i in range(len(lane) - 1):
            ax, ay = lane[i]
            bx, by = lane[i + 1]
            segx = bx - ax
            segy = by - ay
            seg_len2 = segx * segx + segy * segy
            if seg_len2 <= 1e-9:
                continue
            seg_len = math.sqrt(seg_len2)
            t = max(0.0, min(1.0, ((x - ax) * segx + (y - ay) * segy) / seg_len2))
            projx = ax + t * segx
            projy = ay + t * segy
            dx = x - projx
            dy = y - projy
            dist2 = dx * dx + dy * dy
            if best_dist is None or dist2 < best_dist:
                best_dist = dist2
                best_heading = math.atan2(segy, segx)
                cross = segx * (y - ay) - segy * (x - ax)
                best_sign = 1.0 if cross >= 0.0 else -1.0
                best_s = arc_len + t * seg_len
            arc_len += seg_len
        if best_heading is None or best_dist is None:
            return (None, None, None)
        cte = best_sign * math.sqrt(max(best_dist, 0.0))
        theta_e = self._normalize_angle(heading - best_heading)
        return (cte, theta_e, best_s)

    def _update_trajectory_cost_rows(self, traj_msg):
        if not self.show_traj_cost_debug or traj_msg is None:
            return
        stamp = getattr(traj_msg, 'header', None)
        stamp_time = getattr(stamp, 'stamp', None)
        stamp_key = None
        if stamp_time is not None:
            stamp_key = (getattr(stamp_time, 'sec', None), getattr(stamp_time, 'nanosec', None))
        if stamp_key is not None and stamp_key == self._last_traj_stamp:
            return
        self._last_traj_stamp = stamp_key
        rows = []
        timestep = getattr(traj_msg, 'timestep', None)
        waypoints = list(getattr(traj_msg, 'waypoints', []))
        num_waypoints = len(waypoints)
        for idx, wp in enumerate(waypoints):
            x = float(getattr(wp, 'x', 0.0))
            y = float(getattr(wp, 'y', 0.0))
            theta = float(getattr(wp, 'theta', 0.0))
            v = float(getattr(wp, 'v', 0.0))
            tau = float(getattr(wp, 'tau', 0.0))
            cte = float(getattr(wp, 'cte', float('nan')))
            if math.isnan(cte):
                cte = None
            along_track = float(getattr(wp, 'along_track', float('nan')))
            if math.isnan(along_track):
                along_track = None
            costmap_cost = float(getattr(wp, 'costmap_cost', float('nan')))
            if math.isnan(costmap_cost):
                costmap_cost = None
            cte_bad = float(getattr(wp, 'cte_bad', float('nan')))
            if math.isnan(cte_bad):
                cte_bad = None
            obs_bad = float(getattr(wp, 'obs_bad', float('nan')))
            if math.isnan(obs_bad):
                obs_bad = None
            along_track_penalty = float(getattr(wp, 'along_track_penalty', float('nan')))
            if math.isnan(along_track_penalty):
                along_track_penalty = None
            rows.append({
                'idx': idx,
                'x': x,
                'y': y,
                'theta': theta,
                'v': v,
                'tau': tau,
                'cte': cte,
                'along_track': along_track,
                'costmap_cost': costmap_cost,
                'cte_bad': cte_bad,
                'obs_bad': obs_bad,
                'along_track_penalty': along_track_penalty
            })
        self._traj_cost_rows = rows
        self._traj_cost_timestep = timestep

    def _format_debug_value(self, value, width=6, precision=2):
        if value is None or (isinstance(value, float) and math.isnan(value)):
            return f"{'--':>{width}}"
        fmt = f"{{:>{width}.{precision}f}}"
        return fmt.format(value)

    def _draw_local_plan_cost_overlay(self):
        if not self.show_traj_cost_debug:
            return
        probe = self._get_active_costmap_probe()
        if not self._traj_cost_rows and not probe:
            return
        lines = ["Local trajectory cost debug"]
        weight_line = (
            f"w_along={self._cost_weights['along_track']:.1f} "
            f"w_cte={self._cost_weights['cte']:.1f} "
            f"w_costmap={self._cost_weights['costmap']:.1f} "
            f"obs_thr={self._cost_weights['obs_threshold']:.2f} "
            f"cte_thr={self._cost_weights['cte_threshold']:.2f} "
            f"along_thr={self._cost_weights['along_threshold']:.2f}"
        )
        lines.append(weight_line)
        if self._traj_cost_timestep:
            lines.append(f"timestep={self._traj_cost_timestep:.2f}s target_v={self._target_debug_velocity:.2f}m/s")
        else:
            lines.append(f"target_v={self._target_debug_velocity:.2f}m/s")
        if self._traj_cost_rows:
            max_rows = 6
            for row in self._traj_cost_rows[:max_rows]:
                line = (
                    f"{row['idx']:02d} "
                    f"cte:{self._format_debug_value(row['cte'])} "
                    f"along:{self._format_debug_value(row['along_track'])} "
                    f"cost:{self._format_debug_value(row['costmap_cost'], precision=2)} "
                    f"cte_b:{self._format_debug_value(row['cte_bad'], precision=2)} "
                    f"obs_b:{self._format_debug_value(row['obs_bad'], precision=2)} "
                    f"along_p:{self._format_debug_value(row['along_track_penalty'], precision=2)}"
                )
                lines.append(line)
            if len(self._traj_cost_rows) > max_rows:
                lines.append(f"... ({len(self._traj_cost_rows) - max_rows} more)")
        else:
            lines.append("No trajectory samples available")
        if probe and not self.distance_measure_enabled:
            if probe['cost'] is None:
                val_text = "n/a"
            else:
                val_text = f"{probe['cost']:.3f}"
            reason = probe.get('reason')
            note = ""
            if reason == 'out_of_bounds':
                note = " (outside costmap bounds)"
            elif reason == 'no_data':
                note = " (costmap unavailable)"
            elif reason and reason not in ('debug_disabled',):
                note = f" ({reason})"
            lines.append(f"probe x={probe['x']:.2f} y={probe['y']:.2f} cost={val_text}{note}")

        line_height = self.font.get_linesize()
        box_width = max(self.font.size(text)[0] for text in lines) + 16
        box_height = line_height * len(lines) + 8
        overlay = pygame.Surface((box_width, box_height), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 180))
        for i, text in enumerate(lines):
            text_surface = self.font.render(text, True, WHITE)
            overlay.blit(text_surface, (8, 4 + i * line_height))
        self.screen.blit(overlay, (10, SCREEN_HEIGHT - box_height - 10))

    def _draw_costmap_probe_marker(self):
        if not self.show_traj_cost_debug or self.distance_measure_enabled:
            return
        probe = self._get_active_costmap_probe()
        if not probe:
            return
        sx, sy = world_to_screen(probe['x'], probe['y'], self.camera_x, self.camera_y)
        try:
            pygame.draw.circle(self.screen, ORANGE, (sx, sy), 10, 2)
            pygame.draw.circle(self.screen, ORANGE, (sx, sy), 4)
        except Exception:
            return

        label = "n/a" if probe['cost'] is None else f"{probe['cost']:.3f}"
        text_surface = self.font.render(label, True, WHITE)
        padding = 4
        rect = text_surface.get_rect()
        rect.midbottom = (sx, sy - 12)
        bg_rect = rect.inflate(padding * 2, padding * 2)
        overlay = pygame.Surface(bg_rect.size, pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 180))
        overlay.blit(text_surface, (padding, padding))
        self.screen.blit(overlay, bg_rect.topleft)

    def _draw_distance_measurement(self):
        if not self._distance_points:
            return
        try:
            for px, py in self._distance_points:
                sx, sy = world_to_screen(px, py, self.camera_x, self.camera_y)
                pygame.draw.circle(self.screen, YELLOW, (sx, sy), 6, 2)
                pygame.draw.circle(self.screen, YELLOW, (sx, sy), 2)
            if len(self._distance_points) == 2:
                (x1, y1), (x2, y2) = self._distance_points
                sx1, sy1 = world_to_screen(x1, y1, self.camera_x, self.camera_y)
                sx2, sy2 = world_to_screen(x2, y2, self.camera_x, self.camera_y)
                pygame.draw.line(self.screen, YELLOW, (sx1, sy1), (sx2, sy2), 2)
                if self._distance_value is not None:
                    label = f"{self._distance_value:.2f} m"
                    mx = int((sx1 + sx2) / 2)
                    my = int((sy1 + sy2) / 2) - 10
                    text_surface = self.font.render(label, True, WHITE)
                    padding = 4
                    rect = text_surface.get_rect(center=(mx, my))
                    bg_rect = rect.inflate(padding * 2, padding * 2)
                    overlay = pygame.Surface(bg_rect.size, pygame.SRCALPHA)
                    overlay.fill((0, 0, 0, 180))
                    overlay.blit(text_surface, (padding, padding))
                    self.screen.blit(overlay, bg_rect.topleft)
        except Exception:
            return

    def _draw_cost_tuning_panel(self):
        """Render lightweight sliders/text inputs for MPC lane cost tuning."""
        if not self.show_traj_cost_debug:
            self._cost_control_layout = {}
            self._cost_panel_rect = None
            return

        padding = 10
        row_h = 34
        panel_width = 320
        header_h = 26
        total_h = header_h + len(self._cost_controls) * row_h + padding * 2
        x = SCREEN_WIDTH - panel_width - 10
        y = 60
        panel_rect = pygame.Rect(x, y, panel_width, total_h)
        self._cost_panel_rect = panel_rect
        surface = pygame.Surface((panel_rect.width, panel_rect.height), pygame.SRCALPHA)
        surface.fill((0, 0, 0, 150))

        header_txt = self.font.render("Lane cost tuning (live)", True, WHITE)
        surface.blit(header_txt, (padding, padding))
        self._cost_control_layout = {}

        for idx, ctrl in enumerate(self._cost_controls):
            cy = padding + header_h + idx * row_h
            label_txt = self.font.render(ctrl["label"], True, WHITE)
            surface.blit(label_txt, (padding, cy))

            # Slider
            slider_x = padding + 90
            slider_w = 130
            slider_rect = pygame.Rect(slider_x, cy + 6, slider_w, 10)
            pygame.draw.rect(surface, DARK_GRAY, slider_rect)
            pygame.draw.rect(surface, LIGHT_GRAY, slider_rect, width=1)
            t = 0.0
            v_min, v_max = ctrl["min"], ctrl["max"]
            if v_max > v_min:
                t = (ctrl["value"] - v_min) / (v_max - v_min)
                t = max(0.0, min(1.0, t))
            handle_x = slider_rect.x + int(t * slider_rect.w)
            pygame.draw.rect(surface, BLUE, (handle_x - 4, slider_rect.y - 3, 8, slider_rect.h + 6))

            # Text box
            box_w = 70
            input_rect = pygame.Rect(slider_rect.right + 8, cy, box_w, 24)
            active = self._active_input_key == ctrl["key"]
            border_color = BLUE if active else WHITE
            fill_color = (60, 60, 60) if active else (20, 20, 20)
            pygame.draw.rect(surface, fill_color, input_rect)
            pygame.draw.rect(surface, border_color, input_rect, width=1)

            # Prefer live buffer when editing; otherwise show clamped value text
            val_text = f"{ctrl['value']:.3f}" if ctrl["key"] not in ("along_track", "cte", "costmap", "obs_threshold", "cte_threshold", "along_threshold") else f"{ctrl['value']:.1f}"
            if ctrl["key"] == "target_v":
                val_text = f"{ctrl['value']:.2f}"
            if active:
                val_text = self._input_buffer or val_text
            # Build display string with caret
            display_text = val_text
            caret_pos = len(display_text) if not active else self._input_caret
            # Clamp caret
            caret_pos = max(0, min(len(display_text), caret_pos))
            left = display_text[:caret_pos]
            right = display_text[caret_pos:]
            render_text = left + "|" + right if active else display_text

            # Trim to fit box by removing from start if needed
            max_w = input_rect.w - 6
            trimmed = render_text
            while trimmed and self.font.size(trimmed)[0] > max_w:
                # remove first char unless it's caret, keep caret visible by trimming left side
                if trimmed[0] == "|":
                    trimmed = trimmed[1:]
                    caret_pos -= 1
                else:
                    trimmed = trimmed[1:]
            input_txt = self.font.render(trimmed, True, WHITE)
            surface.blit(input_txt, (input_rect.x + 3, input_rect.y + 3))

            self._cost_control_layout[ctrl["key"]] = {
                "slider": pygame.Rect(panel_rect.x + slider_rect.x, panel_rect.y + slider_rect.y, slider_rect.w, slider_rect.h),
                "input": pygame.Rect(panel_rect.x + input_rect.x, panel_rect.y + input_rect.y, input_rect.w, input_rect.h),
            }

        self.screen.blit(surface, (panel_rect.x, panel_rect.y))

    def _handle_cost_tuning_mouse(self, event):
        """Return True if event consumed by tuning UI."""
        if not self.show_traj_cost_debug:
            return False
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self._cost_panel_rect and not self._cost_panel_rect.collidepoint(event.pos):
                # Click outside closes input focus
                self._active_input_key = None
                self._input_caret = 0
                return False
            for key, layout in self._cost_control_layout.items():
                if layout["input"].collidepoint(event.pos):
                    self._active_input_key = key
                    # Prefill buffer with current value
                    cur = next((c["value"] for c in self._cost_controls if c["key"] == key), 0.0)
                    self._input_buffer = f"{cur:.3f}"
                    if key == "target_v":
                        self._input_buffer = f"{cur:.2f}"
                    if key in ("along_track", "cte", "costmap", "obs_threshold", "cte_threshold", "along_threshold"):
                        self._input_buffer = f"{cur:.1f}"
                    self._input_caret = len(self._input_buffer)
                    self._slider_dragging = False
                    return True
                if layout["slider"].collidepoint(event.pos):
                    self._active_slider_key = key
                    self._slider_dragging = True
                    self._update_slider_value_from_pos(event.pos)
                    return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self._slider_dragging:
                self._slider_dragging = False
                self._active_slider_key = None
                return True
        elif event.type == pygame.MOUSEMOTION:
            if self._slider_dragging and self._active_slider_key:
                self._update_slider_value_from_pos(event.pos)
                return True
        return False

    def _update_slider_value_from_pos(self, pos):
        if self._active_slider_key is None:
            return
        layout = self._cost_control_layout.get(self._active_slider_key)
        if not layout:
            return
        slider_rect = layout["slider"]
        t = (pos[0] - slider_rect.x) / float(slider_rect.w)
        t = max(0.0, min(1.0, t))
        ctrl = next((c for c in self._cost_controls if c["key"] == self._active_slider_key), None)
        if ctrl:
            value = ctrl["min"] + t * (ctrl["max"] - ctrl["min"])
            self._set_cost_control_value(ctrl["key"], value)

    def _handle_cost_tuning_key(self, event):
        if not self.show_traj_cost_debug or not self._active_input_key:
            return False
        if event.key == pygame.K_RETURN:
            try:
                val = float(self._input_buffer) if self._input_buffer not in ("", "-", "+") else 0.0
                # Accept out-of-range for manual entry
                self._set_cost_control_value(self._active_input_key, val, clamp=False)
            except Exception:
                pass
            self._active_input_key = None
            self._input_buffer = ""
            self._input_caret = 0
            return True
        if event.key == pygame.K_ESCAPE:
            self._active_input_key = None
            self._input_buffer = ""
            self._input_caret = 0
            return True
        if event.key == pygame.K_BACKSPACE:
            if self._input_caret > 0 and self._input_buffer:
                self._input_buffer = self._input_buffer[:self._input_caret-1] + self._input_buffer[self._input_caret:]
                self._input_caret -= 1
            return True
        if event.key == pygame.K_LEFT:
            self._input_caret = max(0, self._input_caret - 1)
            return True
        if event.key == pygame.K_RIGHT:
            self._input_caret = min(len(self._input_buffer), self._input_caret + 1)
            return True
        if event.key == pygame.K_HOME:
            self._input_caret = 0
            return True
        if event.key == pygame.K_END:
            self._input_caret = len(self._input_buffer)
            return True
        if event.key == pygame.K_a and (event.mod & pygame.KMOD_CTRL):
            # select-all: clear buffer so next input replaces it
            self._input_buffer = ""
            self._input_caret = 0
            return True
        # Accept basic numeric entry
        if event.unicode and (event.unicode.isdigit() or event.unicode in ['.', '-', '+', 'e', 'E']):
            self._input_buffer = (
                self._input_buffer[:self._input_caret] + event.unicode + self._input_buffer[self._input_caret:]
            )
            self._input_caret += 1
            return True
        return False

    # Drawing is centralized in draw.py. Simulator keeps state and updates only.
        
    def handle_map_selector_click(self, pos):
        """Handle clicks on map selector UI elements"""
        if self.dropdown_rect.collidepoint(pos):
            self.dropdown_open = not self.dropdown_open
            return True
            
        if self.dropdown_open:
            # Check dropdown item clicks
            y = self.dropdown_rect.bottom
            for map_name in self.available_maps + ["New Map"]:
                r = pygame.Rect(self.dropdown_rect.x, y, self.dropdown_rect.width, 30)
                if r.collidepoint(pos):
                    if map_name == "New Map":
                        self.load_empty_map()
                    else:
                        self.load_map(os.path.join(self.maps_dir, map_name))
                    self.dropdown_open = False
                    return True
                y += 30
                
        if self.edit_button_rect.collidepoint(pos):
            self.toggle_editor()
            return True

        if getattr(self, 'distance_button_rect', None) and self.distance_button_rect.collidepoint(pos):
            self.distance_measure_enabled = not self.distance_measure_enabled
            self._clear_distance_measurement()
            self._clear_costmap_probe()
            self.dropdown_open = False
            return True
            
        return False
        
    # Editor grid rendering is handled in draw.draw_toolbox / editor.draw which are
    # called from the render loop; keep sim.py focused on state only.
        
    def scan_maps_directory(self):
        """Get list of available map files"""
        try:
            if not os.path.exists(self.maps_dir):
                os.makedirs(self.maps_dir)
            maps = [f for f in os.listdir(self.maps_dir) if f.endswith('.json')]
            return sorted(maps)
        except Exception:
            return []

    def _parse_lane_centerline(self, data):
        """Return list of (x,y) tuples from a lane_centerline field."""
        lane_points = []
        if not data:
            return lane_points
        for pt in data:
            try:
                if isinstance(pt, dict):
                    x = float(pt.get('x', 0.0))
                    y = float(pt.get('y', 0.0))
                elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
                    x = float(pt[0])
                    y = float(pt[1])
                else:
                    continue
                lane_points.append((x, y))
            except Exception:
                continue
        return lane_points

    def load_map(self, map_path):
        """Load a map file and update sim state"""
        try:
            # Clear any previous map state
            self.waypoints = []
            self._waypoint_index = 0
            
            # Load obstacles and scene metadata (start/goal/waypoints)
            self.obstacles, scene_obj = load_map_file(map_path)
            self.current_map = os.path.basename(map_path)

            # Get start pose and update vehicle
            start = scene_obj.get('start', None)
            if start is not None:
                self.vehicle.x = float(start.get('x', 0.0))
                self.vehicle.y = float(start.get('y', 0.0))
                self.vehicle.heading = float(start.get('theta', 0.0))
                self.start_pose = (self.vehicle.x, self.vehicle.y, self.vehicle.heading)
                # Reset grid update pos to current start — initial publish will centre on the car
                self._last_grid_update_pos = (self.vehicle.x, self.vehicle.y)
            else:
                self.start_pose = None
            
            # Get goal pose
            goal = scene_obj.get('goal', None)
            if goal is not None:
                gx = float(goal.get('x', self.vehicle.x + 14.0))
                gy = float(goal.get('y', self.vehicle.y))
                gtheta = float(goal.get('theta', 0.0))
                self.target_pose = (gx, gy, gtheta)
            else:
                self.target_pose = None

            # Load waypoints from scene JSON (if present)
            try:
                wps = scene_obj.get('waypoints', None)
                if wps is None:
                    self.waypoints = []
                else:
                    self.waypoints = [(float(p.get('x', 0.0)), float(p.get('y', 0.0))) for p in wps]
            except Exception:
                self.waypoints = []
            # Publish loaded waypoints as a trajectory
            try:
                if getattr(self, 'pose_publisher', None) is not None:
                    self.pose_publisher.publish_trajectory(self.waypoints)
            except Exception:
                pass

            # Initialize run evaluator for objective scoring
            self.evaluator = RunEvaluator(self.waypoints, self.obstacles, duration=30.0)

            # Clear cumulative lane trail on new map
            try:
                self.pose_publisher.cumulative_lane_trail = []
            except Exception:
                pass
            # Clear fog-of-war state on new map
            if hasattr(self, '_revealed_obstacle_ids'):
                self._revealed_obstacle_ids.clear()
                self._permanent_lane_left.clear()
                self._permanent_lane_right.clear()
                self._permanent_lane_center.clear()

            # Publish updates
            self.pose_publisher.publish_target(gx, gy, 0.0, 0.0, gtheta)

            # Force immediate initial occupancy grid publish so scan_lane() has data from frame 1.
            # Without this, the grid is empty until the car moves 1.5m → planner never starts
            # (map_initialized stays False) → car sits idle for up to 15 seconds.
            try:
                init_obstacles = list(self.obstacles)
                init_cx = self.vehicle.x
                init_cy = self.vehicle.y
                init_heading = self.vehicle.heading
                def _publish_initial_grid():
                    self.pose_publisher.publish_lane_boundaries(
                        init_obstacles, car_x=init_cx, car_y=init_cy,
                        car_heading=init_heading)
                    self.pose_publisher.publish_occupancy_grid_local(
                        init_obstacles, center_x=init_cx, center_y=init_cy,
                        width_m=12.0, height_m=12.0)
                    print("[MAP] Initial occupancy grid + lane boundaries published.")
                import threading as _threading
                _threading.Thread(target=_publish_initial_grid, daemon=True).start()
            except Exception as _e:
                print(f"[MAP] Warning: could not publish initial grid: {_e}")
            
            # Load lane centerlines (from explicit field or from lane obstacles)
            self.lane_centerline = self._parse_lane_centerline(scene_obj.get('lane_centerline'))
            
            # Also extract lanes defined in obstacles marked as is_lane: true
            self.map_lanes = scene_obj.get('lanes', [])
            if self.map_lanes:
                print(f"✓ Loaded {len(self.map_lanes)} lanes from map")
            
            # Load lane objects (polygon obstacles) for the waypoint generator
            self.lane_objects = load_lane_objects_from_json(map_path)
            if self.lane_objects:
                print(f"✓ Loaded {len(self.lane_objects)} lane objects from map")
            
            try:
                self.pose_publisher.publish_lane_centerline(self.lane_centerline)
            except Exception:
                pass
            if self.show_traj_cost_debug:
                self._rebuild_costmap_from_obstacles()
            
        except Exception as e:
            print(f"Failed to load map {map_path}: {e}")
            self.load_empty_map()

    def load_empty_map(self):
        """Load an empty map with default settings"""
        self.obstacles = []
        self.current_map = "New Map"
        # Default positions
        self.vehicle.x = 0.0
        self.vehicle.y = 0.0
        self.vehicle.heading = 0.0
        # For a fresh New Map leave start and goal unset (None)
        self.start_pose = None
        self.target_pose = None
        self.lane_centerline = []
        self.map_lanes = []  # Clear lanes
        self.lane_objects = []  # Clear lane objects
        self.waypoints = []  # Clear waypoints
        # NOTE: Occupancy grid will be computed lazily by the main loop using local window
        # to avoid blocking startup on empty maps. Commenting this out allows instant startup.
        # try:
        #     self.pose_publisher.publish_occupancy_grid(self.obstacles)
        # except Exception:
        #     print("Failed to publish occupancy grid")
        #     pass
        try:
            self.pose_publisher.publish_lane_centerline(self.lane_centerline)
        except Exception:
            pass
        if self.show_traj_cost_debug:
            self._rebuild_costmap_from_obstacles()

    def _compute_grid_background(self):
        """Background thread function for computing occupancy grid without blocking main loop"""
        import time as time_module
        try:
            t_start = time_module.time()
            params = self._grid_compute_params
            if params is None:
                return
            
            # Perform the expensive grid computation in background
            self.pose_publisher.publish_lane_boundaries(
                params.get('all_obstacles', params['obstacles']),
                car_x=params['center_x'],
                car_y=params['center_y'],
                car_heading=params.get('heading', 0.0),
            )
            self.pose_publisher.publish_occupancy_grid_local(
                params['obstacles'],
                center_x=params['center_x'],
                center_y=params['center_y'],
                width_m=12.0,
                height_m=12.0
            )
            
            # Store completion time for reporting
            self._grid_compute_time = (time_module.time() - t_start) * 1000
        except Exception as e:
            print(f"Error in background grid computation: {e}")

    def _update_perception(self):
        """Update fog-of-war: reveal obstacles within perception_range of the car,
        and accumulate lane boundary points permanently."""
        # Always accumulate lane boundary points (so trail is ready when FOW is toggled on)
        try:
            new_left = getattr(self.pose_publisher, 'last_lane_left_pts', [])
            new_right = getattr(self.pose_publisher, 'last_lane_right_pts', [])
            def _add_unique(target, pts, min_dist_sq=0.04):  # 0.2m dedup
                for p in pts:
                    if not any((p[0]-e[0])**2 + (p[1]-e[1])**2 < min_dist_sq for e in target[-200:]):
                        target.append(p)
            _add_unique(self._permanent_lane_left, new_left)
            _add_unique(self._permanent_lane_right, new_right)
            # Only accumulate centerline when both boundaries are visible
            # and each paired point is at least 1.5m apart (genuine lane separation).
            MIN_LANE_SEP = 1.5 * 1.5  # compare squared distances
            if new_left and new_right:
                n = min(len(new_left), len(new_right))
                center_pts = []
                for i in range(n):
                    dsq = (new_left[i][0]-new_right[i][0])**2 + (new_left[i][1]-new_right[i][1])**2
                    if dsq < MIN_LANE_SEP:
                        continue
                    center_pts.append(((new_left[i][0]+new_right[i][0])/2.0,
                                       (new_left[i][1]+new_right[i][1])/2.0))
                _add_unique(self._permanent_lane_center, center_pts)
                if center_pts:
                    self.pose_publisher.publish_persistent_centerline(self._permanent_lane_center)
        except Exception:
            pass

        if not self.perception_mode:
            return
        vx, vy = self.vehicle.x, self.vehicle.y
        r2 = self.perception_range ** 2
        for obs in self.obstacles:
            oid = id(obs)
            if oid in self._revealed_obstacle_ids:
                continue
            # White lane-marking polygons are camera-only — not revealed by lidar.
            # They appear via the permanent lane boundary trail instead.
            if hasattr(obs, 'color'):
                c = obs.color
                if c[0] > 200 and c[1] > 200 and c[2] > 200:
                    continue
            # Reveal obstacle if ANY vertex (or centre for circles) is within 5m
            if hasattr(obs, 'vertices'):
                for (ox, oy) in obs.vertices:
                    if (ox - vx)**2 + (oy - vy)**2 <= r2:
                        self._revealed_obstacle_ids.add(oid)
                        break
            elif hasattr(obs, 'x') and hasattr(obs, 'y'):
                if (obs.x - vx)**2 + (obs.y - vy)**2 <= r2:
                    self._revealed_obstacle_ids.add(oid)

    def toggle_editor(self):
        """Toggle between simulation and editor modes"""
        self.edit_mode = not self.edit_mode
        self.show_map_controls = not self.edit_mode
        
        if self.edit_mode:
            # Initialize editor
            map_path = os.path.join(self.maps_dir, self.current_map) if self.current_map and self.current_map != "New Map" else None
            self.editor = SceneEditor(scene_path=map_path)
            # Share screen with editor
            self.editor.screen = self.screen
            # If editing an existing map, pre-fill the inline save textbox with its name (without extension)
            if self.current_map and self.current_map != "New Map":
                name = os.path.splitext(self.current_map)[0]
                self.editor.save_text = name
            # Sync editor state with current sim state
            self.editor.obstacles = self.obstacles.copy()
            self.editor.camera_x = self.vehicle.x
            self.editor.camera_y = self.vehicle.y
            # sync waypoints into editor
            try:
                self.editor.waypoints = list(getattr(self, 'waypoints', []) or [])
            except Exception:
                self.editor.waypoints = []
            # Sync start/goal into editor so they can be moved
            try:
                self.editor.start_pose = tuple(self.start_pose)
                self.editor.goal_pose = tuple(self.target_pose)
            except Exception:
                pass
        else:
            # Copy obstacles back from editor
            self.obstacles = self.editor.obstacles.copy()
            # Copy waypoints back
            try:
                self.waypoints = list(getattr(self.editor, 'waypoints', []) or [])
                self._waypoint_index = 0
                try:
                    if getattr(self, 'pose_publisher', None) is not None:
                        self.pose_publisher.publish_trajectory(self.waypoints)
                except Exception:
                    pass
            except Exception:
                self.waypoints = []
            # Preserve lane centerline metadata if editor loaded it
            try:
                editor_lane = getattr(self.editor, 'lane_centerline', [])
                self.lane_centerline = self._parse_lane_centerline(editor_lane)
                try:
                    self.pose_publisher.publish_lane_centerline(self.lane_centerline)
                except Exception:
                    pass
            except Exception:
                self.lane_centerline = []
            # Copy start/goal back from editor
            try:
                self.start_pose = tuple(self.editor.start_pose)
                self.target_pose = tuple(self.editor.goal_pose)
                # publish target update
                try:
                    if self.pose_publisher is not None:
                        gx, gy, gtheta = self.target_pose
                        self.pose_publisher.publish_target(gx, gy, 0.0, 0.0, gtheta)
                except Exception:
                    pass
            except Exception:
                pass
            # Clean up
            self.editor = None
            if self.show_traj_cost_debug:
                self._rebuild_costmap_from_obstacles()
            # Update map list
            self.available_maps = self.scan_maps_directory()

    def clear_editor_partial(self):
        """Clear any in-progress editor state (partial polygons, lines, or start/goal placements)."""
        if not self.editor:
            return
        try:
            self.editor.line_start = None
        except Exception:
            pass
        try:
            self.editor.temp_polygon = []
        except Exception:
            pass
        # Clear start/goal placing state
        try:
            self.editor.placing_start = False
            self.editor.placing_start_anchor = None
            self.editor.placing_start_angling = False
        except Exception:
            pass
        try:
            self.editor.placing_goal = False
            self.editor.placing_goal_anchor = None
            self.editor.placing_goal_angling = False
        except Exception:
            pass

    def run(self):
        """Main simulation loop"""
        running = True
        # Compute real delta-time per frame using pygame Clock.tick.
        # We'll call tick at the top of the loop so dt reflects the time
        # since the previous iteration.
        dragging = False
        drag_start = None
        while running:
            # dt in seconds (time elapsed since last call to tick)
            dt_ms = self.clock.tick(FPS)
            dt = dt_ms / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    continue

                # Cost-tuning UI consumes relevant events when debug overlay is visible
                if event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP, pygame.MOUSEMOTION):
                    if self._handle_cost_tuning_mouse(event):
                        continue
                if event.type == pygame.KEYDOWN:
                    if self._handle_cost_tuning_key(event):
                        continue

                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left click
                        pos = event.pos

                        # Debug mode: click to query cost at position
                        if self.debug_mode and not self.edit_mode:
                            wx, wy = screen_to_world(pos[0], pos[1], self.camera_x, self.camera_y, self.camera_zoom)
                            self._debug_click_pos = (wx, wy)

                        # If in edit mode, first allow editor to handle top-right controls
                        if self.edit_mode:
                            # Return button (top-right)
                            if hasattr(self.editor, 'return_rect') and self.editor.return_rect.collidepoint(pos):
                                self.toggle_editor()
                                continue

                            # Save button handling: toggle input or perform save
                            if hasattr(self.editor, 'save_rect') and self.editor.save_rect.collidepoint(pos):
                                # Toggle text-entry for map name, or save if already active.
                                if not self.editor.save_text_active:
                                    # clear any partial items before switching focus
                                    self.clear_editor_partial()
                                    # activate text entry
                                    self.editor.save_text_active = True
                                    # enable faster key repeat for smoother backspace behavior
                                    try:
                                        pygame.key.set_repeat(120, 40)
                                    except Exception:
                                        pass
                                    # ensure textbox has focus and prefill if empty
                                    if not self.editor.save_text:
                                        if self.current_map and self.current_map != "New Map":
                                            self.editor.save_text = os.path.splitext(self.current_map)[0]
                                        else:
                                            self.editor.save_text = ''
                                else:
                                    if self.editor.save_text:
                                        filename = self.editor.save_text if self.editor.save_text.endswith('.json') else f"{self.editor.save_text}.json"
                                        map_path = os.path.join(self.maps_dir, filename)
                                        vehicle_pos = (self.vehicle.x, self.vehicle.y, self.vehicle.heading)
                                        try:
                                            start_for_save = tuple(self.editor.start_pose)
                                        except Exception:
                                            start_for_save = vehicle_pos
                                        try:
                                            goal_for_save = tuple(self.editor.goal_pose)
                                        except Exception:
                                            goal_for_save = tuple(self.target_pose)
                                        if self.editor.save_map(map_path, start_for_save, goal_for_save):
                                            self.current_map = filename
                                            self.available_maps = self.scan_maps_directory()
                                continue

                            # Clicking the toolbox area should not be intercepted by the map selector
                            mx, my = pos
                            # If user clicked inside the save text rect, activate the text input
                            if hasattr(self.editor, 'save_text_rect') and self.editor.save_text_rect.collidepoint(pos):
                                # clear partial items then focus the inline save text box
                                self.clear_editor_partial()
                                self.editor.save_text_active = True
                                try:
                                    pygame.key.set_repeat(120, 40)
                                except Exception:
                                    pass
                                continue

                            for tool, rect in self.editor.toolbox_rects.items():
                                if rect.collidepoint(mx, my):
                                    # If clicking the start_marker/goal_marker tool, start the two-step placement
                                    # clear any partial items before switching tools
                                    self.clear_editor_partial()
                                    if tool == 'start_marker':
                                        # begin two-step start placement (don't remove current marker until finalized)
                                        self.editor.placing_start = True
                                        self.editor.placing_start_anchor = None
                                        self.editor.placing_start_angling = False
                                        self.editor.placing_goal = False
                                        self.editor.selected_tool = tool
                                    elif tool == 'goal_marker':
                                        # begin two-step goal placement (keep existing marker until finalized)
                                        self.editor.placing_goal = True
                                        self.editor.placing_goal_anchor = None
                                        self.editor.placing_goal_angling = False
                                        self.editor.placing_start = False
                                        self.editor.selected_tool = tool
                                    else:
                                        # regular tool selection
                                        self.editor.selected_tool = tool
                                    break
                            else:
                                # Map click: clicking outside toolbox/save should hide text input
                                if hasattr(self.editor, 'save_text_active') and self.editor.save_text_active:
                                    # if click is not on save_rect or save_text_rect, hide input
                                    if not (hasattr(self.editor, 'save_rect') and self.editor.save_rect.collidepoint(pos)) and not (hasattr(self.editor, 'save_text_rect') and self.editor.save_text_rect.collidepoint(pos)):
                                        self.editor.save_text_active = False
                                        try:
                                            pygame.key.set_repeat(0, 0)
                                        except Exception:
                                            pass
                                wx, wy = screen_to_world(mx, my, self.editor.camera_x, self.editor.camera_y, self.editor.camera_zoom)
                                # If remove tool is active, attempt to delete an obstacle or waypoint
                                if self.editor.selected_tool == 'remove':
                                    removed = False
                                    try:
                                        # Prefer removing obstacles first (circle, polygon, line)
                                        for idx, obs in enumerate(list(self.editor.obstacles)):
                                            if isinstance(obs, CircleObstacle):
                                                if self.editor.is_near_point(wx, wy, obs.x, obs.y):
                                                    del self.editor.obstacles[idx]
                                                    removed = True
                                                    break
                                            elif isinstance(obs, PolygonObstacle):
                                                xs = [v[0] for v in obs.vertices]
                                                ys = [v[1] for v in obs.vertices]
                                                if (min(xs) - 0.2) <= wx <= (max(xs) + 0.2) and (min(ys) - 0.2) <= wy <= (max(ys) + 0.2):
                                                    del self.editor.obstacles[idx]
                                                    removed = True
                                                    break
                                            elif isinstance(obs, LineObstacle):
                                                xs = [v[0] for v in obs.vertices]
                                                ys = [v[1] for v in obs.vertices]
                                                if (min(xs) - 0.2) <= wx <= (max(xs) + 0.2) and (min(ys) - 0.2) <= wy <= (max(ys) + 0.2):
                                                    del self.editor.obstacles[idx]
                                                    removed = True
                                                    break
                                    except Exception:
                                        removed = False

                                    if not removed:
                                        # Try removing a waypoint (this will renumber by shifting list indices)
                                        try:
                                            wps = getattr(self.editor, 'waypoints', []) or []
                                            for i, (wxp, wyp) in enumerate(list(wps)):
                                                if self.editor.is_near_point(wx, wy, wxp, wyp):
                                                    # delete and normalize list
                                                    del self.editor.waypoints[i]
                                                    # adjust moving index if needed
                                                    try:
                                                        if getattr(self.editor, 'moving_waypoint_idx', None) is not None:
                                                            if self.editor.moving_waypoint_idx == i:
                                                                self.editor.moving_waypoint_idx = None
                                                            elif self.editor.moving_waypoint_idx > i:
                                                                self.editor.moving_waypoint_idx -= 1
                                                    except Exception:
                                                        self.editor.moving_waypoint_idx = None
                                                    # ensure list contains only tuples and is compact
                                                    try:
                                                        self.editor.waypoints = [(float(px), float(py)) for px, py in self.editor.waypoints]
                                                    except Exception:
                                                        # fallback to empty list on error
                                                        self.editor.waypoints = []
                                                    removed = True
                                                    break
                                        except Exception:
                                            pass
                                    # after a successful remove, publish updated trajectory
                                    if removed:
                                        try:
                                            if getattr(self, 'pose_publisher', None) is not None:
                                                self.pose_publisher.publish_trajectory(getattr(self.editor, 'waypoints', []) or [])
                                        except Exception:
                                            pass
                                    # Done handling remove tool
                                    continue

                                # If waypoint tool is active, place waypoint immediately
                                if self.editor.selected_tool == 'waypoint':
                                    if not hasattr(self.editor, 'waypoints') or self.editor.waypoints is None:
                                        self.editor.waypoints = []
                                    # append new waypoint and normalize list
                                    try:
                                        self.editor.waypoints.append((float(wx), float(wy)))
                                        self.editor.waypoints = [(float(px), float(py)) for px, py in self.editor.waypoints]
                                    except Exception:
                                        # ensure we at least have a valid list
                                        self.editor.waypoints = [(float(wx), float(wy))]
                                    # publish updated trajectory
                                    try:
                                        if getattr(self, 'pose_publisher', None) is not None:
                                            self.pose_publisher.publish_trajectory(getattr(self.editor, 'waypoints', []) or [])
                                    except Exception:
                                        pass
                                else:
                                    # If we're in placing mode, handle two-step anchor+angle
                                    if getattr(self.editor, 'placing_start', False):
                                        # If anchor is not set yet, set anchor (bulb) and enter angling phase
                                        if self.editor.placing_start_anchor is None:
                                            self.editor.placing_start_anchor = (wx, wy)
                                            self.editor.placing_start_angling = True
                                        elif self.editor.placing_start_angling:
                                            # finalize using vector from anchor to this click
                                            ax, ay = self.editor.placing_start_anchor
                                            theta = math.atan2(wy - ay, wx - ax)
                                            self.editor.start_pose = (ax, ay, theta)
                                            self.editor.placing_start = False
                                            self.editor.placing_start_anchor = None
                                            self.editor.placing_start_angling = False
                                    elif getattr(self.editor, 'placing_goal', False):
                                        if self.editor.placing_goal_anchor is None:
                                            self.editor.placing_goal_anchor = (wx, wy)
                                            self.editor.placing_goal_angling = True
                                        elif self.editor.placing_goal_angling:
                                            ax, ay = self.editor.placing_goal_anchor
                                            theta = math.atan2(wy - ay, wx - ax)
                                            self.editor.goal_pose = (ax, ay, theta)
                                            self.editor.placing_goal = False
                                            self.editor.placing_goal_anchor = None
                                            self.editor.placing_goal_angling = False
                                    else:
                                        # Normal tool operations
                                        if self.editor.selected_tool == 'circle':
                                            self.editor.obstacles.append(CircleObstacle(wx, wy, self.editor.obstacle_size))
                                        elif self.editor.selected_tool == 'line':
                                            if self.editor.line_start is None:
                                                self.editor.line_start = (wx, wy)
                                            else:
                                                self.editor.obstacles.append(LineObstacle(self.editor.line_start, (wx, wy), self.editor.obstacle_size))
                                                # Automatically mark line obstacles as lanes
                                                self.editor.lane_obstacle_indices.add(len(self.editor.obstacles) - 1)
                                                self.editor.line_start = None
                                        elif self.editor.selected_tool == 'polygon':
                                            if len(self.editor.temp_polygon) >= 3:
                                                start_x, start_y = self.editor.temp_polygon[0]
                                                if self.editor.is_near_point(wx, wy, start_x, start_y):
                                                    self.editor.obstacles.append(PolygonObstacle(self.editor.temp_polygon, RED))
                                                    self.editor.temp_polygon = []
                                                else:
                                                    self.editor.temp_polygon.append((wx, wy))
                                            else:
                                                self.editor.temp_polygon.append((wx, wy))
                                        elif self.editor.selected_tool == 'select':
                                            self.editor.selected_idx = self.editor.obstacle_at_point(wx, wy)
                                        elif self.editor.selected_tool == 'move':
                                            # Start a move operation: prefer selected obstacle if present
                                            # If clicking near start/goal markers, move them instead
                                            sel_idx = self.editor.obstacle_at_point(wx, wy)
                                            if sel_idx is not None:
                                                self.editor.moving_obs_idx = sel_idx
                                            else:
                                                # Check proximity to start/goal
                                                if self.editor.start_pose is not None and self.editor.is_near_point(wx, wy, self.editor.start_pose[0], self.editor.start_pose[1]):
                                                    self.editor.moving_start = True
                                                elif self.editor.goal_pose is not None and self.editor.is_near_point(wx, wy, self.editor.goal_pose[0], self.editor.goal_pose[1]):
                                                    self.editor.moving_goal = True
                                                else:
                                                    # Check if clicking near a waypoint to move it
                                                    moved_wp = None
                                                    try:
                                                        for i, (wxp, wyp) in enumerate(getattr(self.editor, 'waypoints', []) or []):
                                                            if self.editor.is_near_point(wx, wy, wxp, wyp):
                                                                moved_wp = i
                                                                break
                                                    except Exception:
                                                        moved_wp = None
                                                    if moved_wp is not None:
                                                        self.editor.moving_waypoint_idx = moved_wp
                                                    else:
                                                        # Otherwise start panning drag
                                                        self.editor.dragging = True
                                                        self.editor.drag_last = (mx, my)
                            # Done handling edit-mode click
                            continue

                        # Not in edit mode (or edit mode didn't consume click) - check map selector and edit button
                        if self.handle_map_selector_click(pos):
                            continue
                        if self.distance_measure_enabled:
                            wx, wy = screen_to_world(pos[0], pos[1], self.camera_x, self.camera_y)
                            self._record_distance_point(wx, wy)
                            continue
                        if self.show_traj_cost_debug:
                            wx, wy = screen_to_world(pos[0], pos[1], self.camera_x, self.camera_y)
                            cost_val, reason = self._lookup_costmap_cost(wx, wy)
                            self._record_costmap_probe(wx, wy, cost_val, reason)
                            try:
                                publisher = getattr(self, 'pose_publisher', None)
                                logger = publisher.get_logger() if publisher and hasattr(publisher, 'get_logger') else None
                                if logger is not None and hasattr(logger, 'info'):
                                    if cost_val is None:
                                        if reason == 'out_of_bounds':
                                            logger.info(f'Costmap probe at ({wx:.2f}, {wy:.2f}) is outside planner costmap bounds')
                                        elif reason:
                                            logger.info(f'Costmap probe at ({wx:.2f}, {wy:.2f}) unavailable ({reason})')
                                        else:
                                            logger.info(f'Costmap probe at ({wx:.2f}, {wy:.2f}) unavailable')
                                    else:
                                        logger.info(f'Costmap probe at ({wx:.2f}, {wy:.2f}) = {cost_val:.4f}')
                            except Exception:
                                pass
                            continue
                            
                    elif event.button == 3:  # Right click
                        if self.edit_mode:
                            # Complete polygon
                            if len(self.editor.temp_polygon) >= 3:
                                self.editor.obstacles.append(PolygonObstacle(self.editor.temp_polygon))
                            self.editor.temp_polygon = []
                            self.editor.line_start = None
                        else:
                            if self.distance_measure_enabled:
                                self._clear_distance_measurement()
                    
                elif event.type == pygame.MOUSEBUTTONUP:
                    if self.edit_mode:
                        # Finish any move operations
                        self.editor.dragging = False
                        self.editor.drag_last = None
                        self.editor.selected_idx = None
                        self.editor.moving_obs_idx = None
                        self.editor.moving_start = False
                        self.editor.moving_goal = False
                        self.editor.moving_waypoint_idx = None
                    
                elif event.type == pygame.MOUSEMOTION:
                    if self.edit_mode:  
                        if event.buttons[0]:  # Left button
                            if self.editor.selected_tool == 'move':
                                # If moving an obstacle by index
                                if getattr(self.editor, 'moving_obs_idx', None) is not None:
                                    idx = self.editor.moving_obs_idx
                                    wx, wy = screen_to_world(event.pos[0], event.pos[1], self.editor.camera_x, self.editor.camera_y, self.editor.camera_zoom)
                                    obs = self.editor.obstacles[idx]
                                    if isinstance(obs, CircleObstacle):
                                        obs.x = wx
                                        obs.y = wy
                                    elif isinstance(obs, PolygonObstacle) or isinstance(obs, LineObstacle):
                                        # translate all vertices by the delta
                                        # compute previous world pos of mouse from drag_last
                                        if self.editor.drag_last is not None:
                                            prev_wx, prev_wy = screen_to_world(self.editor.drag_last[0], self.editor.drag_last[1], self.editor.camera_x, self.editor.camera_y, self.editor.camera_zoom)
                                            dx = wx - prev_wx
                                            dy = wy - prev_wy
                                            obs.vertices = [(x + dx, y + dy) for x, y in obs.vertices]
                                    # update drag_last for continuous movement
                                    self.editor.drag_last = event.pos
                                elif self.editor.moving_start:
                                    wx, wy = screen_to_world(event.pos[0], event.pos[1], self.editor.camera_x, self.editor.camera_y, self.editor.camera_zoom)
                                    try:
                                        _, _, th = self.editor.start_pose
                                    except Exception:
                                        th = 0.0
                                    self.editor.start_pose = (wx, wy, th)
                                elif self.editor.moving_goal:
                                    wx, wy = screen_to_world(event.pos[0], event.pos[1], self.editor.camera_x, self.editor.camera_y, self.editor.camera_zoom)
                                    try:
                                        _, _, th = self.editor.goal_pose
                                    except Exception:
                                        th = 0.0
                                    self.editor.goal_pose = (wx, wy, th)
                                elif self.editor.dragging:
                                    dx = (event.pos[0] - self.editor.drag_last[0]) / SCALE
                                    dy = -(event.pos[1] - self.editor.drag_last[1]) / SCALE
                                    self.editor.camera_x -= dx
                                    self.editor.camera_y -= dy
                                    self.editor.drag_last = event.pos
                                elif getattr(self.editor, 'moving_waypoint_idx', None) is not None:
                                    wx, wy = screen_to_world(event.pos[0], event.pos[1], self.editor.camera_x, self.editor.camera_y, self.editor.camera_zoom)
                                    try:
                                        idx = self.editor.moving_waypoint_idx
                                        self.editor.waypoints[idx] = (wx, wy)
                                    except Exception:
                                        pass
                            elif self.editor.selected_tool == 'select' and self.editor.selected_idx is not None:
                                # Move selected obstacle
                                wx, wy = screen_to_world(event.pos[0], event.pos[1], self.editor.camera_x, self.editor.camera_y, self.editor.camera_zoom)
                                obs = self.editor.obstacles[self.editor.selected_idx]
                                if isinstance(obs, CircleObstacle):
                                    obs.x = wx
                                    obs.y = wy
                
                elif event.type == pygame.KEYDOWN:
                    if self.editor and self.editor.save_text_active:
                        if event.key == pygame.K_RETURN:
                            if self.editor.save_text:  # Only save if there's text entered
                                # Add .json if not present
                                filename = self.editor.save_text if self.editor.save_text.endswith('.json') else f"{self.editor.save_text}.json"
                                map_path = os.path.join(self.maps_dir, filename)
                                
                                # Save map using editor's method (prefer editor start/goal)
                                vehicle_pos = (self.vehicle.x, self.vehicle.y, self.vehicle.heading)
                                try:
                                    start_for_save = tuple(self.editor.start_pose)
                                except Exception:
                                    start_for_save = vehicle_pos
                                try:
                                    goal_for_save = tuple(self.editor.goal_pose)
                                except Exception:
                                    goal_for_save = tuple(self.target_pose)
                                if self.editor.save_map(map_path, start_for_save, goal_for_save):
                                    # Update current map and available maps
                                    self.current_map = filename
                                    self.available_maps = self.scan_maps_directory()
                            self.editor.save_text_active = False
                            try:
                                pygame.key.set_repeat(0, 0)
                            except Exception:
                                pass
                        elif event.key == pygame.K_BACKSPACE:
                            self.editor.save_text = self.editor.save_text[:-1]
                        elif event.key == pygame.K_ESCAPE:
                            self.editor.save_text_active = False
                            try:
                                pygame.key.set_repeat(0, 0)
                            except Exception:
                                pass
                        else:
                            if event.unicode.isprintable():
                                self.editor.save_text += event.unicode
                        continue
                    if event.key == pygame.K_SPACE and not self.edit_mode:
                        if getattr(self, '_auto_follow_locked', False):
                            # In automated eval mode, SPACE is disabled to prevent accidental toggling.
                            pass
                        else:
                            self.follow_planner = not self.follow_planner
                            # Log status about follower presence when toggling follow mode
                            try:
                                if self.follow_planner:
                                    if self.pose_publisher.is_follower_connected():
                                        self.pose_publisher.get_logger().info('Follow mode enabled and external follower appears connected')
                                    else:
                                        self.pose_publisher.get_logger().info('Follow mode enabled but no external follower detected (manual control remains)')
                                else:
                                    self.pose_publisher.get_logger().info('Follow mode disabled; manual control active')
                            except Exception:
                                pass
                    elif event.key == pygame.K_u and not self.edit_mode:
                        # Toggle Pure Pursuit controller
                        self.use_pure_pursuit = not self.use_pure_pursuit
                        try:
                            state = 'enabled' if self.use_pure_pursuit else 'disabled'
                            self.pose_publisher.get_logger().info(f'Pure Pursuit controller {state}')
                        except Exception:
                            pass
                    elif event.key == pygame.K_t and not self.edit_mode:
                        self.show_traj_cost_debug = not self.show_traj_cost_debug
                        self._active_input_key = None
                        self._slider_dragging = False
                        self._active_slider_key = None
                        self._clear_costmap_probe()
                        if self.show_traj_cost_debug:
                            self._rebuild_costmap_from_obstacles()
                        else:
                            self._traj_cost_rows = []
                        try:
                            state = 'enabled' if self.show_traj_cost_debug else 'disabled'
                            self.pose_publisher.get_logger().info(f'Local trajectory cost debug overlay {state}')
                        except Exception:
                            pass
<<<<<<< HEAD
                    elif event.key == pygame.K_v and not self.edit_mode:
                        self.perception_mode = not self.perception_mode
                        if not self.perception_mode:
                            self._revealed_obstacle_ids.clear()
                            self._permanent_lane_left.clear()
                            self._permanent_lane_right.clear()
                        print(f"[VIZ] Perception/fog-of-war mode {'ON' if self.perception_mode else 'OFF'}")
                    elif event.key == pygame.K_d and not self.edit_mode:
                        self.debug_mode = not self.debug_mode
                        self._debug_click_pos = None
                        print(f"[DEBUG] Heatmap mode {'ON (click to query cost)' if self.debug_mode else 'OFF'}")
=======
                    elif event.key == pygame.K_g and not self.edit_mode:
                        self._show_planner_ghosts = not self._show_planner_ghosts
                        try:
                            state = 'enabled' if self._show_planner_ghosts else 'disabled'
                            self.pose_publisher.get_logger().info(f'Planner ghost render {state}')
                        except Exception:
                            pass
>>>>>>> refs/remotes/origin/lane-planner
                    elif event.key == pygame.K_ESCAPE and self.edit_mode:
                        # Cancel current tool operation
                        self.editor.temp_polygon = []
                        self.editor.line_start = None
                    elif event.key == pygame.K_l and self.edit_mode and self.editor.selected_tool == 'select':
                        # Toggle lane status for selected polygon (L key in select mode)
                        if self.editor.selected_idx is not None:
                            obs = self.editor.obstacles[self.editor.selected_idx]
                            if isinstance(obs, PolygonObstacle):
                                if self.editor.selected_idx in self.editor.lane_obstacle_indices:
                                    self.editor.lane_obstacle_indices.remove(self.editor.selected_idx)
                                    print(f"Polygon {self.editor.selected_idx} is now a REGULAR obstacle")
                                else:
                                    self.editor.lane_obstacle_indices.add(self.editor.selected_idx)
                                    print(f"Polygon {self.editor.selected_idx} is now a LANE")

            try:
                rclpy.spin_once(self.pose_publisher, timeout_sec=0)
            except Exception:
                pass
            if self.show_traj_cost_debug:
                self._refresh_costmap_from_ros()

            speed_input, steer_input = self.handle_input()
            try:
                traj_msg = self.pose_publisher.latest_trajectory_msg
            except Exception:
                traj_msg = None

            self._update_trajectory_cost_rows(traj_msg)
            now_time = pygame.time.get_ticks() / 1000.0

<<<<<<< HEAD
            # In auto-follow locked mode, force follow_planner True every frame.
            if getattr(self, '_auto_follow_locked', False):
                self.follow_planner = True

            latest_ack = None
            target_speed = 0.0
            desired_steer = 0.0

=======
            target_speed = self.vehicle.speed
            desired_steer = self.vehicle.steering_angle
            latest_ack = None
>>>>>>> refs/remotes/origin/lane-planner
            if self.follow_planner:
                # When follow_planner is enabled, the simulator accepts external
                # AckermannDrive commands from an external follower. If such a
                # command is present, it overrides keyboard inputs.
                latest_ack = self.pose_publisher.get_latest_ack()
                if latest_ack is not None:
                    age = self.pose_publisher.ack_age_seconds()
                    if age is not None and age > 1.0:
                        # stale message — treat as no command
                        latest_ack = None
                    else:
                        target_speed = getattr(latest_ack, 'speed', 0.0)
                        desired_steer = getattr(latest_ack, 'steering_angle', 0.0)
<<<<<<< HEAD
=======
                # else: no external command -> keep keyboard inputs (manual driving)
>>>>>>> refs/remotes/origin/lane-planner

            # if using traj follower, force input, else do normal input
            if self.follow_planner and latest_ack is not None:
                self.vehicle.force_update(dt, target_speed, desired_steer)
            else:
                self.vehicle.update(dt, speed_input, steer_input)

            colliding_obstacles = self.collision_detector.check_collision(self.vehicle.get_corners(), self.obstacles)
            self.is_colliding = len(colliding_obstacles) > 0

            # Tick the run evaluator — only once the planner is actually driving
            try:
                ev = getattr(self, 'evaluator', None)
                if ev and not ev.finished:
                    mode = getattr(self.pose_publisher, 'planner_mode', 'GPS')
                    wp_idx = getattr(self.pose_publisher, 'planner_waypoint_idx', 0)
                    # Don't start the eval clock until the planner has sent at least one command.
                    # This avoids counting idle startup time (planner takes ~5s to initialize).
                    planner_live = (latest_ack is not None or ev.start_time is not None)
                    if planner_live:
                        ev.tick(self.vehicle, self.collision_detector, planner_mode=mode, wp_index=max(0, wp_idx))
                        if ev.finished:
                            print(ev.summary(), flush=True)
                            if getattr(self, '_auto_quit_on_eval', False):
                                self._shutdown_sim()
                                import sys; sys.exit(0)
                    else:
                        if getattr(self, '_auto_follow_locked', False):
                            print(f"[EVAL] Waiting for planner... (elapsed wall: {(pygame.time.get_ticks()/1000):.0f}s)", flush=True) if (pygame.time.get_ticks() % 3000) < 50 else None
            except Exception:
                pass

            self.camera_x = self.vehicle.x
            self.camera_y = self.vehicle.y

            # Update local occupancy grid if vehicle has moved enough (asynchronous, non-blocking)
            try:
                now = pygame.time.get_ticks() / 1000.0
                
                # Check if background grid thread is done
                with self._grid_thread_lock:
                    if self._grid_thread is not None and not self._grid_thread.is_alive():
                        self._grid_thread.join()
                        self._grid_thread = None
                        self._last_grid_update_pos = self._grid_compute_params.get('pos', self._last_grid_update_pos)
                    
                    # Only start a new publish when the vehicle has moved enough
                    if self._grid_thread is None:
                        dist_moved = math.hypot(
                            self.vehicle.x - self._last_grid_update_pos[0],
                            self.vehicle.y - self._last_grid_update_pos[1]
                        )
                        if dist_moved > self._grid_update_threshold:
                            # In fog-of-war mode, occupancy grid only contains
                            # obstacles within perception range (simulates lidar FoV).
                            # Lane boundaries use all obstacles (camera range filter
                            # is built into publish_lane_boundaries).
                            if self.perception_mode:
                                vx, vy = self.vehicle.x, self.vehicle.y
                                r2 = self.perception_range ** 2
                                grid_obs = []
                                for obs in self.obstacles:
                                    if hasattr(obs, 'vertices'):
                                        if any((ox - vx)**2 + (oy - vy)**2 <= r2 for ox, oy in obs.vertices):
                                            grid_obs.append(obs)
                                    elif hasattr(obs, 'x') and hasattr(obs, 'y'):
                                        if (obs.x - vx)**2 + (obs.y - vy)**2 <= r2:
                                            grid_obs.append(obs)
                            else:
                                grid_obs = list(self.obstacles)
                            self._grid_compute_params = {
                                'obstacles': grid_obs,
                                'all_obstacles': list(self.obstacles),  # for lane boundaries
                                'center_x': self.vehicle.x,
                                'center_y': self.vehicle.y,
                                'heading':  self.vehicle.heading,
                                'pos': (self.vehicle.x, self.vehicle.y),
                            }
                            self._grid_thread = threading.Thread(target=self._compute_grid_background, daemon=True)
                            self._grid_thread.start()
            except Exception as e:
                print(f"Error in grid update logic: {e}")

            # Periodically publish the simulated pose
            if now_time - self._last_publish_time >= self.publish_interval:
                try:
                    self.pose_publisher.publish_pose(self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.vehicle.speed, self.vehicle.steering_angle)
                    try:
                        self.pose_publisher.publish_odometry(self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.vehicle.speed)
                    except Exception:
                        pass
                    # Publish lane boundaries on every pose tick (cheap) so planner
                    # always has boundary data aligned with the current heading.
                    try:
                        self.pose_publisher.publish_lane_boundaries(
                            list(self.obstacles),
                            car_x=self.vehicle.x,
                            car_y=self.vehicle.y,
                            car_heading=self.vehicle.heading,
                        )
                    except Exception:
                        pass
                    # Publish waypoints once (on map load, already done in load_scene)
                    # Don't re-publish every cycle — it resets the planner mission.
                except Exception as e:
                    print(f"Failed to publish pose: {e}")
                self._last_publish_time = now_time

            # Update fog-of-war perception state
            self._update_perception()

            # Rendering
            self.screen.fill(GRAY)

            if self.edit_mode:
                # Use SceneEditor's draw (toolbox draws return/save controls)
                self.editor.screen = self.screen  # Share screen surface
                self.editor.draw()
            else:
                # Draw simulation view (all rendering centralized in draw.py)
                draw_grid(self.screen, self.camera_x, self.camera_y, zoom=self.camera_zoom)
                draw_lane_centerline(self.screen, self.lane_centerline or [], self.vehicle, self.camera_x, self.camera_y, zoom=self.camera_zoom)
                # call draw_vehicle from draw.py directly
                try:
                    draw_vehicle(self.screen, self.vehicle, self.camera_x, self.camera_y, is_colliding=self.is_colliding, zoom=self.camera_zoom)
                except Exception:
                    pass
                
                # Draw RL gap selector waypoints if active

                
                # HUD
                draw_hud(self.screen, self.vehicle, self.font, self.follow_planner, self.use_pure_pursuit,
                         planner_mode=getattr(self.pose_publisher, 'planner_mode', 'GPS'),
                         evaluator=getattr(self, 'evaluator', None),
                         perception_mode=self.perception_mode,
                         debug_mode=self.debug_mode)
                # Obstacles and start/goal
                if self.perception_mode:
                    visible_obs = [o for o in self.obstacles if id(o) in self._revealed_obstacle_ids]
                    draw_obstacles(self.screen, visible_obs, self.camera_x, self.camera_y, zoom=self.camera_zoom)
                    # Draw perception range circle
                    sx, sy = world_to_screen(self.vehicle.x, self.vehicle.y, self.camera_x, self.camera_y, self.camera_zoom)
                    pr = int(self.perception_range * SCALE * self.camera_zoom)
                    pygame.draw.circle(self.screen, (100, 100, 100), (sx, sy), pr, 1)
                    # Draw permanent lane boundary trails
                    draw_permanent_lanes(self.screen, self._permanent_lane_left, self._permanent_lane_right,
                                         self.camera_x, self.camera_y, self.camera_zoom)
                    draw_permanent_centerline(self.screen, self._permanent_lane_center,
                                              self.camera_x, self.camera_y, self.camera_zoom)
                else:
                    draw_obstacles(self.screen, self.obstacles, self.camera_x, self.camera_y, zoom=self.camera_zoom)
                draw_start_goal(self.screen, self.start_pose, self.target_pose, self.camera_x, self.camera_y, zoom=self.camera_zoom)
                self._draw_costmap_probe_marker()
<<<<<<< HEAD
                
=======
                self._draw_distance_measurement()
>>>>>>> refs/remotes/origin/lane-planner
                # Draw waypoints in sim view as numbered green circles
                try:
                    for i, (x, y) in enumerate(getattr(self, 'waypoints', []) or []):
                        sx, sy = world_to_screen(x, y, self.camera_x, self.camera_y, self.camera_zoom)
                        radius = 12
                        pygame.draw.circle(self.screen, GREEN, (sx, sy), radius)
                        try:
                            label = str(i + 1)
                            txt = self.font.render(label, True, BLACK)
                            txt_rect = txt.get_rect(center=(sx, sy))
                            self.screen.blit(txt, txt_rect)
                        except Exception:
                            pygame.draw.circle(self.screen, BLACK, (sx, sy), 3)
                except Exception:
                    pass
                # Planner trajectory and detected lane centerline
                draw_lane_boundaries(self.screen, self)
                draw_cumulative_lane_trail(self.screen, self)
                draw_planner_trajectory(self.screen, self)
<<<<<<< HEAD
                draw_detected_lane_cl(self.screen, self)
                draw_mpc_targets(self.screen, self)
                self._draw_local_plan_cost_overlay()
                # Debug heatmap + click-to-query label
                if self.debug_mode:
                    draw_debug_heatmap(self.screen, self)
                    if self._debug_click_pos is not None:
                        draw_debug_cost_label(self.screen, self.font,
                                              self._debug_click_pos[0], self._debug_click_pos[1], self)
                
=======
                if self._show_planner_ghosts:
                    draw_planner_ghosts(self.screen, self)
                self._draw_local_plan_cost_overlay()

            self._draw_cost_tuning_panel()
>>>>>>> refs/remotes/origin/lane-planner
            draw_map_selector(self.screen, self)  # Always show map selector

            pygame.display.flip()
        self._shutdown_sim()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--auto-follow', action='store_true',
                        help='Auto-enable follow_planner without pressing SPACE')
    parser.add_argument('--eval-duration', type=float, default=30.0,
                        help='Evaluation duration in seconds (auto-quit when done)')
    parser.add_argument('--fog', action='store_true',
                        help='Start with fog-of-war/perception mode ON')
    parser.add_argument('map', nargs='?', default=None, help='Map file path')
    args = parser.parse_args()
    simulator = Simulator(scene_arg=args.map)
    if args.fog:
        simulator.perception_mode = True
        print('[VIZ] Fog-of-war mode ON (press V to toggle)')
    if args.auto_follow:
        simulator.follow_planner = True
        simulator._auto_follow_locked = True
        simulator._auto_quit_on_eval = True
        simulator.evaluator.duration = args.eval_duration
        print(f'[EVAL] Auto-follow enabled, evaluating for {args.eval_duration:.0f}s (timer starts when planner connects)')
    simulator.run()
