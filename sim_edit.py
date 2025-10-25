import pygame
import math
import json
import os
from objects import CircleObstacle, PolygonObstacle, LineObstacle
from obstacle_loader import load_obstacles_from_json
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
        
        if scene_path:
            try:
                loaded = load_obstacles_from_json(scene_path)
                self.obstacles = loaded
            except Exception:
                # ignore and start empty
                pass

        # default tool: move (1)
        self.selected_tool = 'move'  # move, select, circle, line, polygon
        self.selected_idx = None
        self.temp_polygon = []
        self.line_start = None
        self.obstacle_size = cone_radius
        self.toolbox_rects = {}
        self.panning = False
        self.save_text = ""
        self.save_text_active = False
        # start/goal for editor (None means not set)
        self.start_pose = None
        self.goal_pose = None
        self.placing_start = False
        self.placing_goal = False
        
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

    def world_to_screen(self, x, y):
        return world_to_screen(x, y, self.camera_x, self.camera_y)

    def screen_to_world(self, sx, sy):
        return screen_to_world(sx, sy, self.camera_x, self.camera_y)
        
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
            
            # Create directories if needed
            maps_dir = os.path.dirname(map_path)
            if maps_dir and not os.path.exists(maps_dir):
                os.makedirs(maps_dir, exist_ok=True)
            
            # Write atomically using temporary file
            tmp_path = map_path + '.tmp'
            with open(tmp_path, 'w') as f:
                json.dump(scene, f, indent=2)
            os.replace(tmp_path, map_path)
            
            # Update editor state
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

        # Temp polygon preview
        if len(self.temp_polygon) > 0:
            pts = [self.world_to_screen(x, y) for x, y in self.temp_polygon]
            if len(pts) > 1:
                pygame.draw.lines(self.screen, YELLOW, False, pts, 2)
            for p in pts:
                pygame.draw.circle(self.screen, YELLOW, p, 4)

        # Draw toolbox and controls
        draw_toolbox(self.screen, self)
        pygame.display.flip()

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
