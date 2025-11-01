import pygame
import math
import json
import os
from objects import CircleObstacle, PolygonObstacle, LineObstacle
from sim_map_loader import load_map_file
from constants import *
from draw import *

class SceneEditor:
    def __init__(self, scene_path=None):
        self.scene_path = scene_path
        self.screen = None  # Set by parent
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('monospace', 18)
        self.camera_x = 0
        self.camera_y = 0
        self.dragging = False
        self.drag_last = None
        self.obstacles = []
        # list of waypoint tuples [(x,y), ...] or empty list
        self.waypoints = []
        
        if scene_path:
            try:
                loaded, scene_obj = load_map_file(scene_path)
                self.obstacles = loaded
            except Exception:
                # ignore and start empty; ensure scene_obj exists for downstream
                scene_obj = {}
                self.obstacles = []

            # attempt to load waypoints from scene JSON (if present)
            try:
                wps = scene_obj.get('waypoints', None)
                if wps is None:
                    self.waypoints = []
                else:
                    self.waypoints = [(float(p.get('x', 0.0)), float(p.get('y', 0.0))) for p in wps]
            except Exception:
                # ignore missing/invalid waypoints
                self.waypoints = []

        # default tool: move (1)
        self.selected_tool = 'move'  # move, select, circle, line, polygon
        self.selected_idx = None
        self.temp_polygon = []
        self.line_start = None
        self.obstacle_size = cone_radius
        self.toolbox_rects = {}
        self.panning = False
        # Text-input map naming (inline save box to left of Save button)
        self.save_text = ""
        self.save_text_active = False
        # start/goal for editor (None means not set)
        self.start_pose = None
        self.goal_pose = None
        self.placing_start = False
        self.placing_goal = False
        # Two-step placement state: anchor (position) and angling phase
        self.placing_start_anchor = None
        self.placing_start_angling = False
        self.placing_goal_anchor = None
        self.placing_goal_angling = False
        self.preview_theta = 0.0
        # Move tool state for translating objects
        self.moving_start = False
        self.moving_goal = False
        self.moving_obs_idx = None
        # waypoint move state: index being moved or None
        self.moving_waypoint_idx = None
        
        # Top-right buttons setup
        padding = 10
        btn_w = 80
        btn_h = 30
        text_w = 200
            
        # Position from right side
        x = SCREEN_WIDTH - padding - btn_w
        y = padding
            
        # Return button on far right
        self.return_rect = pygame.Rect(x, y, btn_w, btn_h)
            
        # Save button to left of return
        x -= padding + btn_w
        self.save_rect = pygame.Rect(x, y, btn_w, btn_h)
            
        # Text input to left of save
        x -= padding + text_w
        self.save_text_rect = pygame.Rect(x, y, text_w, btn_h)
        
        # Initialize text input state
        self.save_text_placeholder = "Map Name"
            
    def is_near_point(self, x1, y1, x2, y2, threshold=0.5):
        """Check if two points are within threshold distance"""
        return math.hypot(x1 - x2, y1 - y2) <= threshold
        
    def save_map(self, map_path, vehicle_pos=None, target_pos=None):
        """Save current scene to a map file.
        
        Args:
            map_path: Path where to save the map file
            vehicle_pos: Optional external start position (x,y,theta). If None, uses editor's start_pose
            target_pos: Optional external goal position (x,y,theta). If None, uses editor's goal_pose
        """
        try:
            # Create scene dict with start/goal positions
            scene = {}
            
            # Use provided positions or internal editor state
            start = vehicle_pos if vehicle_pos is not None else self.start_pose
            goal = target_pos if target_pos is not None else self.goal_pose
            
            # Convert to dict format
            if start is not None:
                sx, sy, st = start
                scene['start'] = {'x': sx, 'y': sy, 'theta': st}
            else:
                scene['start'] = None
                
            if goal is not None:
                gx, gy, gt = goal
                scene['goal'] = {'x': gx, 'y': gy, 'theta': gt}
            else:
                scene['goal'] = None
            
            # Add obstacles with full properties
            scene['obstacles'] = []
            for obs in self.obstacles:
                if isinstance(obs, CircleObstacle):
                    scene['obstacles'].append({
                        'type': 'circle',
                        'x': obs.x,
                        'y': obs.y,
                        'radius': obs.radius
                    })
                elif isinstance(obs, PolygonObstacle):
                    scene['obstacles'].append({
                        'type': 'polygon',
                        'vertices': obs.vertices,
                        'color': obs.color
                    })
                elif isinstance(obs, LineObstacle):
                    v = obs.vertices
                    scene['obstacles'].append({
                        'type': 'line',
                        'start': v[0],
                        'end': v[2],
                        'width': math.hypot(v[0][0]-v[1][0], v[0][1]-v[1][1]),
                        'color': obs.color
                    })

            # Waypoints (list of {x,y}) or null when empty
            if getattr(self, 'waypoints', None) and len(self.waypoints) > 0:
                scene['waypoints'] = [{'x': float(x), 'y': float(y)} for x, y in self.waypoints]
            else:
                scene['waypoints'] = None
            
            # Create directories if needed
            maps_dir = os.path.dirname(map_path)
            if maps_dir and not os.path.exists(maps_dir):
                os.makedirs(maps_dir, exist_ok=True)
            
            # Write directly to the requested path (non-atomic per user request)
            with open(map_path, 'w') as f:
                json.dump(scene, f, indent=2)

            # Update editor state and log
            print(f"Saved map to {map_path}")
            self.save_text = ""  # Clear the text input
            self.save_text_active = False  # Hide the text input
            self.scene_path = map_path
            return True
            
        except Exception as e:
            print(f"Failed to save map {map_path}: {e}")
            return False

    def draw(self):
        # Clear screen
        self.screen.fill(GRAY)

        # Grid
        draw_grid(self.screen, self.camera_x, self.camera_y)

        # Obstacles
        draw_obstacles(self.screen, self.obstacles, self.camera_x, self.camera_y)

        # Start/Goal (shared util)
        try:
            draw_start_goal(self.screen, self.start_pose, self.goal_pose, self.camera_x, self.camera_y)
        except Exception:
            pass

        # Waypoints: draw numbered green circles (1..N)
        try:
            for i, (x, y) in enumerate(getattr(self, 'waypoints', []) or []):
                sx, sy = world_to_screen(x, y, self.camera_x, self.camera_y)
                radius = 12
                # circle background
                pygame.draw.circle(self.screen, GREEN, (sx, sy), radius)
                # waypoint number (1-based)
                try:
                    label = str(i + 1)
                    txt = self.font.render(label, True, BLACK)
                    txt_rect = txt.get_rect(center=(sx, sy))
                    self.screen.blit(txt, txt_rect)
                except Exception:
                    # if font render fails, fallback to small dot
                    pygame.draw.circle(self.screen, BLACK, (sx, sy), 3)
        except Exception:
            pass

        # Temp polygon preview
        if len(self.temp_polygon) > 0:
            pts = [world_to_screen(x, y, self.camera_x, self.camera_y) for x, y in self.temp_polygon]
            if len(pts) > 1:
                pygame.draw.lines(self.screen, YELLOW, False, pts, 2)
            for p in pts:
                pygame.draw.circle(self.screen, YELLOW, p, 4)

        # Line preview: if a line start exists but the line isn't finalized, draw preview to mouse
        if self.line_start is not None:
            try:
                mx, my = pygame.mouse.get_pos()
                wx, wy = screen_to_world(mx, my, self.camera_x, self.camera_y)
                start_screen = world_to_screen(self.line_start[0], self.line_start[1], self.camera_x, self.camera_y)
                end_screen = world_to_screen(wx, wy, self.camera_x, self.camera_y)
                pygame.draw.line(self.screen, WHITE, start_screen, end_screen, 2)
                pygame.draw.circle(self.screen, WHITE, start_screen, 4)
            except Exception:
                pass

        # If in angling mode for start/goal show a preview: bulb at anchor and line to mouse cursor
        try:
            mx, my = pygame.mouse.get_pos()
            wx, wy = screen_to_world(mx, my, self.camera_x, self.camera_y)
            if getattr(self, 'placing_start_angling', False) and self.placing_start_anchor is not None:
                ax, ay = self.placing_start_anchor
                a_screen = world_to_screen(ax, ay, self.camera_x, self.camera_y)
                pygame.draw.circle(self.screen, START_COLOR, a_screen, 8)
                # compute capped preview point based on ANGLE_PREVIEW_LENGTH
                dx = wx - ax
                dy = wy - ay
                dist = math.hypot(dx, dy)
                if dist > 1e-6:
                    scale = min(ANGLE_PREVIEW_LENGTH, dist) / dist
                    px = ax + dx * scale
                    py = ay + dy * scale
                else:
                    px, py = ax + ANGLE_PREVIEW_LENGTH, ay
                cursor_screen = world_to_screen(px, py, self.camera_x, self.camera_y)
                pygame.draw.line(self.screen, START_COLOR, a_screen, cursor_screen, 2)
            if getattr(self, 'placing_goal_angling', False) and self.placing_goal_anchor is not None:
                ax, ay = self.placing_goal_anchor
                a_screen = world_to_screen(ax, ay, self.camera_x, self.camera_y)
                pygame.draw.circle(self.screen, GOAL_COLOR, a_screen, 8)
                dx = wx - ax
                dy = wy - ay
                dist = math.hypot(dx, dy)
                if dist > 1e-6:
                    scale = min(ANGLE_PREVIEW_LENGTH, dist) / dist
                    px = ax + dx * scale
                    py = ay + dy * scale
                else:
                    px, py = ax + ANGLE_PREVIEW_LENGTH, ay
                cursor_screen = world_to_screen(px, py, self.camera_x, self.camera_y)
                pygame.draw.line(self.screen, GOAL_COLOR, a_screen, cursor_screen, 2)
        except Exception:
            pass

        # Draw toolbox and controls (parent is responsible for flipping)
        draw_toolbox(self.screen, self)

    def obstacle_at_point(self, wx, wy):
        for idx, obs in enumerate(self.obstacles):
            if isinstance(obs, CircleObstacle):
                if (obs.x-wx)**2 + (obs.y-wy)**2 <= (obs.radius+0.2)**2:
                    return idx
            elif isinstance(obs, PolygonObstacle):
                xs = [v[0] for v in obs.vertices]
                ys = [v[1] for v in obs.vertices]
                if min(xs)-0.2<=wx<=max(xs)+0.2 and min(ys)-0.2<=wy<=max(ys)+0.2:
                    return idx
        return None
