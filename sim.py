import pygame
import math
import os
import json
import rclpy
from objects import CircleObstacle, CollisionDetector, PolygonObstacle, LineObstacle
from obstacle_loader import load_obstacles_from_json
from constants import *
from sim_publisher import VehiclePublisher
from sim_edit import SceneEditor
from draw import *

pygame.init()

class Vehicle:
    def __init__(self, x=0, y=0):
        # Vehicle parameters in meters
        self.wheelbase = 0.4572*0.8  # L (18 inches)
        self.track_width = 0.3048  # W (12 inches)
        self.length = 0.9144 # 3 feet in meters
        self.width = 0.6096  # 2 feet in meters

        self.max_speed = 2.2352  # m/s (5 mph)
        self.throttle_acceleration = 2.5  # m/s^2
        self.steering_rate = math.radians(45) # rad/s

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

    def update(self, dt, speed_input, steer_input):
        """Update vehicle state w/ bicycle model"""

        # update speed based on throttle input
        self.speed = max(-self.max_speed, min(self.max_speed, self.speed + speed_input * self.throttle_acceleration * dt))
        if speed_input < 1e-6:
            # natural deceleration
            self.speed *= 0.98

        # update effective steering angle based on steering input
        if steer_input != 0:
            self.steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, self.steering_angle + steer_input * self.steering_rate * dt))
        else:
            self.steering_angle = self.steering_angle * 0.9 # natural return to center
        # update position and heading
        if abs(self.steering_angle) > 1e-6:
            turning_radius = self.wheelbase / math.tan(self.steering_angle)
            angular_velocity = self.speed / turning_radius
        else: angular_velocity = 0

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
        return math.radians(33.58) # 30 deg, 38 deg

class Simulator:
    def __init__(self, scene_arg: str = None):
        rclpy.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Vehicle Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 18)
        
        # UI state for map selector
        self.maps_dir = os.path.join(os.path.dirname(__file__), 'maps')
        self.available_maps = self.scan_maps_directory()
        self.current_map = None
        self.dropdown_open = False
        self.dropdown_rect = pygame.Rect(SCREEN_WIDTH - 220, 10, 150, 30)
        self.edit_button_rect = pygame.Rect(SCREEN_WIDTH - 60, 10, 50, 30)
        
        # Editor state
        self.edit_mode = False
        self.editor = None  # Will be initialized when entering edit mode
        self.show_map_controls = True  # Toggle for dropdown/edit visibility
        
        # Initialize publisher before loading maps
        self.pose_publisher = VehiclePublisher()
        
        # load scene from config if available (obstacles, start, goal)
        self.vehicle = Vehicle()
        self.vehicle.heading = 0.0
        self.vehicle.speed = 0.0

        # Load initial map
        if scene_arg and os.path.exists(scene_arg):
            self.load_map(scene_arg)
        elif len(self.available_maps) > 0:
            self.load_map(os.path.join(self.maps_dir, self.available_maps[0]))
        else:
            self.load_empty_map()

        # collision detector used each frame
        self.collision_detector = CollisionDetector()
        self.is_colliding = False

        self.camera_x = 0
        self.camera_y = 0


        self.active_trajectory = None
        self.traj_index = 0
        self.follow_planner = False

        try:
            self.pose_publisher.publish_occupancy_grid(self.obstacles, width_m=30.0, height_m=30.0, resolution=0.1)
        except Exception as e:
            print(f"Failed to publish initial occupancy grid: {e}")

        try:
            # publish stuff
            self.pose_publisher.publish_pose(self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.vehicle.speed, self.vehicle.steering_angle)
            self.pose_publisher.publish_odometry(self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.vehicle.speed)
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

        # TODO: add keys to change car dimensions (or have them be input at start of program?)

        return speed_input, steer_input

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

    def load_map(self, map_path):
        """Load a map file and update sim state"""
        try:
            # Load obstacles
            self.obstacles = load_obstacles_from_json(map_path)
            self.current_map = os.path.basename(map_path)
            
            # Load start/goal and update vehicle
            with open(map_path, 'r') as fh:
                scene_obj = json.load(fh)
                
            # Get start pose and update vehicle
            start = scene_obj.get('start', None)
            if start is not None:
                self.vehicle.x = float(start.get('x', 0.0))
                self.vehicle.y = float(start.get('y', 0.0))
                self.vehicle.heading = float(start.get('theta', 0.0))
                self.start_pose = (self.vehicle.x, self.vehicle.y, self.vehicle.heading)
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
            
            # Publish updates
            self.pose_publisher.publish_occupancy_grid(self.obstacles)
            self.pose_publisher.publish_target(gx, gy, 0.0, 0.0, gtheta)
            
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
        # Publish empty grid
        try:
            self.pose_publisher.publish_occupancy_grid(self.obstacles)
        except Exception:
            pass

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
            # Sync start/goal into editor so they can be moved
            try:
                self.editor.start_pose = tuple(self.start_pose)
                self.editor.goal_pose = tuple(self.target_pose)
            except Exception:
                pass
        else:
            # Copy obstacles back from editor
            self.obstacles = self.editor.obstacles.copy()
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
        dt = 1.0 / FPS
        dragging = False
        drag_start = None

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left click
                        pos = event.pos

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
                                wx, wy = screen_to_world(mx, my, self.editor.camera_x, self.editor.camera_y)
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
                                                # Otherwise start panning drag
                                                self.editor.dragging = True
                                                self.editor.drag_last = (mx, my)
                            # Done handling edit-mode click
                            continue

                        # Not in edit mode (or edit mode didn't consume click) - check map selector and edit button
                        if self.handle_map_selector_click(pos):
                            continue
                            
                    elif event.button == 3:  # Right click
                        if self.edit_mode:
                            # Complete polygon
                            if len(self.editor.temp_polygon) >= 3:
                                self.editor.obstacles.append(PolygonObstacle(self.editor.temp_polygon))
                            self.editor.temp_polygon = []
                            self.editor.line_start = None
                            
                elif event.type == pygame.MOUSEBUTTONUP:
                    if self.edit_mode:
                        # Finish any move operations
                        self.editor.dragging = False
                        self.editor.drag_last = None
                        self.editor.selected_idx = None
                        self.editor.moving_obs_idx = None
                        self.editor.moving_start = False
                        self.editor.moving_goal = False
                    
                elif event.type == pygame.MOUSEMOTION:
                    if self.edit_mode:  
                        if event.buttons[0]:  # Left button
                            if self.editor.selected_tool == 'move':
                                # If moving an obstacle by index
                                if getattr(self.editor, 'moving_obs_idx', None) is not None:
                                    idx = self.editor.moving_obs_idx
                                    wx, wy = screen_to_world(event.pos[0], event.pos[1], self.editor.camera_x, self.editor.camera_y)
                                    obs = self.editor.obstacles[idx]
                                    if isinstance(obs, CircleObstacle):
                                        obs.x = wx
                                        obs.y = wy
                                    elif isinstance(obs, PolygonObstacle) or isinstance(obs, LineObstacle):
                                        # translate all vertices by the delta
                                        # compute previous world pos of mouse from drag_last
                                        if self.editor.drag_last is not None:
                                            prev_wx, prev_wy = screen_to_world(self.editor.drag_last[0], self.editor.drag_last[1], self.editor.camera_x, self.editor.camera_y)
                                            dx = wx - prev_wx
                                            dy = wy - prev_wy
                                            obs.vertices = [(x + dx, y + dy) for x, y in obs.vertices]
                                    # update drag_last for continuous movement
                                    self.editor.drag_last = event.pos
                                elif self.editor.moving_start:
                                    wx, wy = screen_to_world(event.pos[0], event.pos[1], self.editor.camera_x, self.editor.camera_y)
                                    try:
                                        _, _, th = self.editor.start_pose
                                    except Exception:
                                        th = 0.0
                                    self.editor.start_pose = (wx, wy, th)
                                elif self.editor.moving_goal:
                                    wx, wy = screen_to_world(event.pos[0], event.pos[1], self.editor.camera_x, self.editor.camera_y)
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
                            elif self.editor.selected_tool == 'select' and self.editor.selected_idx is not None:
                                # Move selected obstacle
                                wx, wy = screen_to_world(event.pos[0], event.pos[1], self.editor.camera_x, self.editor.camera_y)
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
                    elif event.key == pygame.K_ESCAPE and self.edit_mode:
                        # Cancel current tool operation
                        self.editor.temp_polygon = []
                        self.editor.line_start = None

            try:
                rclpy.spin_once(self.pose_publisher, timeout_sec=0)
            except Exception:
                pass

            speed_input, steer_input = self.handle_input()
            try:
                traj_msg = self.pose_publisher.latest_trajectory_msg
            except Exception:
                traj_msg = None

            if self.follow_planner:
                # When follow_planner is enabled, the simulator accepts external
                # AckermannDrive commands from an external follower. If such a
                # command is present, it overrides keyboard inputs. If not,
                # manual keyboard driving remains active (do not attempt internal
                # trajectory following).
                latest_ack = None
                try:
                    latest_ack = self.pose_publisher.get_latest_ack()
                except Exception:
                    latest_ack = None

                if latest_ack is not None:
                    # Only accept ack commands if they are recent (avoid stale control).
                    age = None
                    try:
                        age = self.pose_publisher.ack_age_seconds()
                    except Exception:
                        age = None
                    if age is not None and age > 1.0:
                        # message too old; ignore
                        try:
                            self.pose_publisher.get_logger().info(f'Ignoring stale AckermannDrive (age={age:.2f}s)')
                        except Exception:
                            pass
                    else:
                        # Map AckermannDrive -> simulator control inputs
                        target_speed = getattr(latest_ack, 'speed', 0.0)
                    speed_delta = target_speed - self.vehicle.speed
                    if abs(speed_delta) < 1e-3:
                        speed_input = 0
                    else:
                        speed_input = max(-1.0, min(1.0, speed_delta / (self.vehicle.throttle_acceleration * dt)))

                        desired_steer = getattr(latest_ack, 'steering_angle', 0.0)
                        steer_delta = desired_steer - self.vehicle.steering_angle
                        if abs(steer_delta) < 1e-3:
                            steer_input = 0
                        else:
                            steer_input = max(-1.0, min(1.0, steer_delta / (self.vehicle.steering_rate * dt)))
                # else: no external command -> keep keyboard inputs (manual driving)

            self.vehicle.update(dt,speed_input,steer_input)

            colliding_obstacles = self.collision_detector.check_collision(self.vehicle.get_corners(), self.obstacles)
            self.is_colliding = len(colliding_obstacles) > 0

            self.camera_x = self.vehicle.x
            self.camera_y = self.vehicle.y

            # Periodically publish the simulated pose
            now = pygame.time.get_ticks() / 1000.0
            if now - self._last_publish_time >= self.publish_interval:
                try:
                    self.pose_publisher.publish_pose(self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.vehicle.speed, self.vehicle.steering_angle)
                    try:
                        self.pose_publisher.publish_odometry(self.vehicle.x, self.vehicle.y, self.vehicle.heading, self.vehicle.speed)
                    except Exception:
                        pass
                except Exception as e:
                    print(f"Failed to publish pose: {e}")
                self._last_publish_time = now

            # Rendering
            self.screen.fill(GRAY)
            
            if self.edit_mode:
                # Use SceneEditor's draw (toolbox draws return/save controls)
                self.editor.screen = self.screen  # Share screen surface
                self.editor.draw()
            else:
                # Draw simulation view (all rendering centralized in draw.py)
                draw_grid(self.screen, self.camera_x, self.camera_y)
                # call draw_vehicle from draw.py directly
                try:
                    draw_vehicle(self.screen, self.vehicle, self.camera_x, self.camera_y, is_colliding=self.is_colliding)
                except Exception:
                    pass
                # HUD
                draw_hud(self.screen, self.vehicle, self.font, self.follow_planner)
                # Obstacles and start/goal
                draw_obstacles(self.screen, self.obstacles, self.camera_x, self.camera_y)
                draw_start_goal(self.screen, self.start_pose, self.target_pose, self.camera_x, self.camera_y)
                # Planner trajectory
                draw_planner_trajectory(self.screen, self)
                
            draw_map_selector(self.screen, self)  # Always show map selector

            pygame.display.flip()
            self.clock.tick(FPS)
        pygame.quit()
        self.pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    simulator = Simulator()
    simulator.run()