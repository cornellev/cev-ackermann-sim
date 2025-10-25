import pygame
import math
import os
import json
import rclpy
from rclpy.node import Node
from objects import CircleObstacle, CollisionDetector, PolygonObstacle, LineObstacle
from obstacle_loader import load_obstacles_from_json
from constants import *
from sim_publisher import VehiclePublisher
from cev_msgs.msg import Trajectory as CevTrajectory
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

    def world_to_screen(self, x, y):
        return world_to_screen(x, y, self.camera_x, self.camera_y)

    def draw_grid(self):
        draw_grid(self.screen, self.camera_x, self.camera_y)

    def draw_wheel(self, center_x, center_y, width, length, wheel_angle):
        """Draws a single wheel as a rectangle"""
        half_length = length / 2
        half_width = width / 2

        unrotated_corners = [
            (-half_length, -half_width),
            (-half_length, half_width),
            (half_length, half_width),
            (half_length, -half_width),
        ]
        world_corners = []
        for x_local, y_local in unrotated_corners:
            x_world = center_x + x_local * math.cos(wheel_angle) - y_local * math.sin(wheel_angle)
            y_world = center_y + x_local * math.sin(wheel_angle) + y_local * math.cos(wheel_angle)
            world_corners.append((x_world, y_world))

        screen_corners = [self.world_to_screen(x, y) for x, y in world_corners]
        pygame.draw.polygon(self.screen, BLACK, screen_corners)

    def draw_vehicle(self):
        """Draws the vehicle as a polygon."""
        corners = self.vehicle.get_corners()
        screen_corners = [self.world_to_screen(x, y) for x, y in corners]
        vehicle_color = RED if self.is_colliding else LIGHT_BLUE
        pygame.draw.polygon(self.screen, vehicle_color, screen_corners)
        # Draw a line to indicate the front of the vehicle
        front_mid_x = (corners[0][0] + corners[1][0]) / 2
        front_mid_y = (corners[0][1] + corners[1][1]) / 2
        center_screen = self.world_to_screen(self.vehicle.x, self.vehicle.y)
        front_screen = self.world_to_screen(front_mid_x, front_mid_y)
        pygame.draw.line(self.screen, YELLOW, center_screen, front_screen, 3)

        # draw wheels of the vehicle
        wheel_width = 0.07 # meters
        wheel_length = 0.2 # meters
        half_wheelbase = self.vehicle.wheelbase / 2
        half_track = self.vehicle.track_width / 2
        # Back wheels (left and right) - aligned with vehicle heading
        for side in [-1, 1]:  # left = 1, right = -1
            back_wheel_x = self.vehicle.x - half_wheelbase * math.cos(self.vehicle.heading) + side * half_track * math.cos(self.vehicle.heading + math.pi/2)
            back_wheel_y = self.vehicle.y - half_wheelbase * math.sin(self.vehicle.heading) + side * half_track * math.sin(self.vehicle.heading + math.pi/2)
            self.draw_wheel(back_wheel_x, back_wheel_y, wheel_width, wheel_length, self.vehicle.heading)

        # derivation from https://www.mathworks.com/help/vdynblks/ref/steeringsystem.html
        right_wheel_angle = math.atan((self.vehicle.wheelbase*math.tan(self.vehicle.steering_angle))/(self.vehicle.wheelbase + (self.vehicle.track_width/2)*math.tan(self.vehicle.steering_angle)))
        left_wheel_angle = math.atan((self.vehicle.wheelbase*math.tan(self.vehicle.steering_angle))/(self.vehicle.wheelbase - (self.vehicle.track_width/2)*math.tan(self.vehicle.steering_angle)))
        # Front wheels (left and right) - turned by steering angle
        for side, wheel_angle in zip([-1, 1], [right_wheel_angle, left_wheel_angle]):
            front_wheel_x = self.vehicle.x + half_wheelbase * math.cos(self.vehicle.heading) + side * half_track * math.cos(self.vehicle.heading + math.pi/2)
            front_wheel_y = self.vehicle.y + half_wheelbase * math.sin(self.vehicle.heading) + side * half_track * math.sin(self.vehicle.heading + math.pi/2)
            self.draw_wheel(front_wheel_x, front_wheel_y, wheel_width, wheel_length, self.vehicle.heading + wheel_angle)
    def draw_obstacles(self):
        draw_obstacles(self.screen, self.obstacles, self.camera_x, self.camera_y)

    def draw_hud(self):
        """Displays vehicle state information on the screen."""
        speed_kmh = self.vehicle.speed * 3.6
        steer_deg = math.degrees(self.vehicle.steering_angle)

        info = [
            f"Speed: {speed_kmh:.1f} km/h",
            f"Steering: {steer_deg:.1f} degrees",
            f"Position: ({self.vehicle.x:.1f}, {self.vehicle.y:.1f}) m",
            f"Heading: {math.degrees(self.vehicle.heading):.1f} degrees"
        ]

        for i, line in enumerate(info):
            text_surface = self.font.render(line, True, WHITE)
            self.screen.blit(text_surface, (10, 10 + i * 25))
        
        # added rendering for following
        follow_text = f"Follow planner: {'ON' if self.follow_planner else 'OFF'}"
        follow_surface = self.font.render(follow_text, True, (0, 255, 0) if self.follow_planner else (255, 100, 100))
        self.screen.blit(follow_surface, (10, 10 + len(info) * 25))
    
    def draw_targets(self):
        draw_start_goal(self.screen, self.start_pose, self.target_pose, self.camera_x, self.camera_y)

    # this is probably really really slow but I'll worry about that later
    def draw_map_selector(self):
        """Draw the map selector dropdown and edit button"""
        if not self.show_map_controls:
            return
            
        # Draw dropdown button
        pygame.draw.rect(self.screen, (200, 200, 200), self.dropdown_rect)
        # Remove .json from display name
        name = self.current_map.replace('.json', '') if self.current_map else "Select Map"
        text = self.font.render(name[:18], True, (0, 0, 0))
        self.screen.blit(text, (self.dropdown_rect.x + 5, self.dropdown_rect.y + 5))
        
        # Draw dropdown list if open
        if self.dropdown_open:
            y = self.dropdown_rect.bottom
            for map_name in self.available_maps + ["New Map"]:
                r = pygame.Rect(self.dropdown_rect.x, y, self.dropdown_rect.width, 30)
                pygame.draw.rect(self.screen, (255, 255, 255), r)
                pygame.draw.rect(self.screen, (180, 180, 180), r, 1)
                # Remove .json from display name
                display_name = map_name.replace('.json', '') if map_name != "New Map" else map_name
                text = self.font.render(display_name[:18], True, (0, 0, 0))
                self.screen.blit(text, (r.x + 5, r.y + 5))
                y += 30
                
        # Draw edit button
        pygame.draw.rect(self.screen, (100, 149, 237), self.edit_button_rect)
        text = self.font.render("Edit", True, (255, 255, 255))
        text_rect = text.get_rect(center=self.edit_button_rect.center)
        self.screen.blit(text, text_rect)
        
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
        
    def draw_editor_grid(self):
        # Use shared draw_grid for editor
        self.editor.screen = self.screen
        draw_grid(self.screen, self.editor.camera_x, self.editor.camera_y)
            
    def world_to_screen(self, x, y):
        return world_to_screen(x, y, self.camera_x, self.camera_y)
        
    def screen_to_world(self, sx, sy):
        return screen_to_world(sx, sy, self.camera_x, self.camera_y)
        
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

    def draw_planner_trajectory(self):
        """Draw the latest planner trajectory (or active trajectory) as a yellow polyline with waypoint markers."""
        try:
            traj_msg = None
            if getattr(self, 'active_trajectory', None) is not None:
                traj_msg = self.active_trajectory
            else:
                traj_msg = getattr(self.pose_publisher, 'latest_trajectory_msg', None)
        except Exception:
            traj_msg = None

        if traj_msg is None:
            return

        points = []
        try:
            for wp in traj_msg.waypoints:
                points.append(self.world_to_screen(wp.x, wp.y))
        except Exception:
            return

        if len(points) == 0:
            return

        try:
            pygame.draw.lines(self.screen, YELLOW, False, points, 2)
        except Exception:
            pass

        for p in points:
            try:
                pygame.draw.circle(self.screen, YELLOW, p, 4)
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
                        # Always check map selector first
                        if self.handle_map_selector_click(event.pos):
                            continue
                            
                        if self.edit_mode:
                            pos = event.pos

                            # Return button (top-right)
                            if hasattr(self.editor, 'return_rect') and self.editor.return_rect.collidepoint(pos):
                                self.toggle_editor()
                                continue

                            # Save button handling: toggle input or perform save
                            if hasattr(self.editor, 'save_rect') and self.editor.save_rect.collidepoint(pos):
                                if not self.editor.save_text_active:
                                    self.editor.save_text_active = True
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

                            # Clicking elsewhere should hide text input
                            self.editor.save_text_active = False

                            # Pass the click to the editor toolbox / map
                            mx, my = event.pos
                            for tool, rect in self.editor.toolbox_rects.items():
                                if rect.collidepoint(mx, my):
                                    self.editor.selected_tool = tool
                                    break
                            else:
                                # Map click: handle tool actions
                                wx, wy = self.editor.screen_to_world(mx, my)
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
                                    self.editor.dragging = True
                                    self.editor.drag_last = (mx, my)
                            
                    elif event.button == 3:  # Right click
                        if self.edit_mode:
                            # Complete polygon
                            if len(self.editor.temp_polygon) >= 3:
                                self.editor.obstacles.append(PolygonObstacle(self.editor.temp_polygon))
                            self.editor.temp_polygon = []
                            self.editor.line_start = None
                            
                elif event.type == pygame.MOUSEBUTTONUP:
                    if self.edit_mode:
                        self.editor.dragging = False
                        self.editor.drag_last = None
                        self.editor.selected_idx = None
                    
                elif event.type == pygame.MOUSEMOTION:
                    if self.edit_mode:  
                        if event.buttons[0]:  # Left button
                            if self.editor.selected_tool == 'move' and self.editor.dragging:
                                dx = (event.pos[0] - self.editor.drag_last[0]) / SCALE
                                dy = -(event.pos[1] - self.editor.drag_last[1]) / SCALE
                                self.editor.camera_x -= dx
                                self.editor.camera_y -= dy
                                self.editor.drag_last = event.pos
                            elif self.editor.selected_tool == 'select' and self.editor.selected_idx is not None:
                                # Move selected obstacle
                                wx, wy = self.editor.screen_to_world(*event.pos)
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
                        elif event.key == pygame.K_BACKSPACE:
                            self.editor.save_text = self.editor.save_text[:-1]
                        elif event.key == pygame.K_ESCAPE:
                            self.editor.save_text_active = False
                        else:
                            if event.unicode.isprintable():
                                self.editor.save_text += event.unicode
                        continue
                    
                    if event.key == pygame.K_SPACE and not self.edit_mode:
                        self.follow_planner = not self.follow_planner
                        if self.follow_planner:
                            try:
                                traj_msg = self.pose_publisher.latest_trajectory_msg
                            except Exception:
                                traj_msg = None
                            
                            if traj_msg is not None:
                                self.active_trajectory = traj_msg
                                self.active_traj_stamp = traj_msg.header.stamp
                                self.traj_index = 0
                            else:
                                self.active_trajectory = None
                                self.traj_index = 0
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

            if self.follow_planner and traj_msg is not None:
                if self.active_trajectory is None or traj_msg is not None and getattr(traj_msg, 'header', None) is not None and (self.active_trajectory is None or traj_msg.header.stamp != self.active_traj_stamp):
                    self.active_trajectory = traj_msg
                    self.active_traj_stamp = traj_msg.header.stamp
                    self.traj_index = 0

                if self.active_trajectory is not None and len(self.active_trajectory.waypoints) > 0 and self.traj_index < len(self.active_trajectory.waypoints):
                    wp = self.active_trajectory.waypoints[self.traj_index]
                    tx = wp.x
                    ty = wp.y
                    tv = wp.v

                    dx = tx - self.vehicle.x
                    dy = ty - self.vehicle.y
                    dist = math.hypot(dx, dy)

                    desired_heading = math.atan2(dy, dx)
                    heading_error = (desired_heading - self.vehicle.heading + math.pi) % (2 * math.pi) - math.pi

                    kp = 1.0
                    desired_steer = max(-self.vehicle.max_steering_angle, min(self.vehicle.max_steering_angle, kp * heading_error))
                    steer_delta = desired_steer - self.vehicle.steering_angle
                    if abs(steer_delta) < 1e-3:
                        steer_input = 0
                    else:
                        steer_input = max(-1.0, min(1.0, steer_delta / (self.vehicle.steering_rate * dt)))

                    if tv is None:
                        tv = self.vehicle.max_speed * 0.5
                    if dist < 1.0:
                        speed_scale = 0.3
                    else:
                        speed_scale = 1.0

                    desired_speed = max(-self.vehicle.max_speed, min(self.vehicle.max_speed, tv))
                    speed_delta = desired_speed - self.vehicle.speed
                    if abs(speed_delta) < 1e-3:
                        speed_input = 0
                    else:
                        speed_input = max(-1.0, min(1.0, (speed_delta / self.vehicle.throttle_acceleration) / dt)) * speed_scale

                    if dist < 0.5:
                        self.traj_index += 1
                else:
                    self.active_trajectory = None
                    self.traj_index = 0

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
                # Use SceneEditor's draw
                self.editor.screen = self.screen  # Share screen surface
                self.editor.draw()
                
                # Draw Save and Return buttons
                # Return button
                pygame.draw.rect(self.screen, LIGHT_BLUE, self.editor.return_rect)
                text = self.font.render('Return', True, BLACK)
                self.screen.blit(text, (self.editor.return_rect.x + 10, self.editor.return_rect.y + 4))
                
                # Save button
                pygame.draw.rect(self.screen, GREEN, self.editor.save_rect)
                text = self.font.render('Save', True, BLACK)
                self.screen.blit(text, (self.editor.save_rect.x + 20, self.editor.save_rect.y + 4))
            else:
                # Draw simulation view
                self.draw_grid()
                self.draw_vehicle()
                self.draw_hud()
                self.draw_obstacles()
                self.draw_targets()
                self.draw_planner_trajectory()
                
            draw_map_selector(self.screen, self)  # Always show map selector

            pygame.display.flip()
            self.clock.tick(FPS)
        pygame.quit()
        self.pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    simulator = Simulator()
    simulator.run()