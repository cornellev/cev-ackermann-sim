import pygame
import math
import json
import os
from objects import CircleObstacle, PolygonObstacle, LineObstacle
from obstacle_loader import load_obstacles_from_json
from constants import *
from draw_utils import *

# Use tkinter file dialogs for Open / Save As. Wrap in try/except so editor still runs
try:
    import tkinter as tk
    from tkinter import filedialog
except Exception:
    tk = None
    filedialog = None

# pygame is initialized by the main simulator when embedded

class SceneEditor:
    def __init__(self, scene_path=None, embedded=False):
        self.scene_path = scene_path
        if not embedded:
            self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
            pygame.display.set_caption('Scene Editor')
        else:
            self.screen = None  # Will be set by parent
            
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('monospace', 18)
        self.camera_x = 0
        self.camera_y = 0
        self.dragging = False
        self.drag_last = None
        self.obstacles = []
        self.embedded = embedded
        
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
        
        # Embedded mode buttons
        if embedded:
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
            # (top-right) Return and Save remain; Start/Goal moved to bottom toolbox
            
            # Text input to left of save (only shown when active)
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
        
    def save_map(self, map_path, vehicle_pos, target_pos):
        """Save current scene to a map file"""
        scene = {}
        if vehicle_pos is not None:
            scene['start'] = {'x': vehicle_pos[0], 'y': vehicle_pos[1], 'theta': vehicle_pos[2]}
        else:
            scene['start'] = None
        if target_pos is not None:
            scene['goal'] = {'x': target_pos[0], 'y': target_pos[1], 'theta': target_pos[2]}
        else:
            scene['goal'] = None
        scene['obstacles'] = [obs.to_dict() for obs in self.obstacles]
        try:
            maps_dir = os.path.dirname(map_path)
            if maps_dir and not os.path.exists(maps_dir):
                os.makedirs(maps_dir, exist_ok=True)
            # write atomically
            tmp_path = map_path + '.tmp'
            with open(tmp_path, 'w') as f:
                json.dump(scene, f, indent=2)
            os.replace(tmp_path, map_path)
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

        # Start/Goal with direction preview
        mx, my = pygame.mouse.get_pos()
        wx, wy = self.screen_to_world(mx, my)

        # Draw start/goal with direction preview
        if self.placing_start and self.start_pose and len(self.start_pose) == 4:
            x, y, _, _ = self.start_pose
            theta = math.atan2(wy - y, wx - x)
            preview_pose = (x, y, theta)
            draw_start_goal(self.screen, preview_pose, self.goal_pose, self.camera_x, self.camera_y)
        elif self.placing_goal and self.goal_pose and len(self.goal_pose) == 4:
            x, y, _, _ = self.goal_pose
            theta = math.atan2(wy - y, wx - x)
            preview_pose = (x, y, theta)
            draw_start_goal(self.screen, self.start_pose, preview_pose, self.camera_x, self.camera_y)
        else:
            draw_start_goal(self.screen, self.start_pose, self.goal_pose, self.camera_x, self.camera_y)

        # Temp polygon preview
        if len(self.temp_polygon) > 0:
            pts = [self.world_to_screen(x, y) for x, y in self.temp_polygon]
            if len(pts) > 1:
                pygame.draw.lines(self.screen, YELLOW, False, pts, 2)
            for p in pts:
                pygame.draw.circle(self.screen, YELLOW, p, 4)

        # Toolbox (bottom) + top-right embedded controls
        from draw_utils import draw_toolbox
        draw_toolbox(self.screen, self)

        # Standalone top-right controls are still handled in this class
        if not self.embedded:
            padding = 10
            btn_w = 80
            btn_h = 28
            x = SCREEN_WIDTH - padding - btn_w
            y = padding

            # Exit button
            self.exit_rect = pygame.Rect(x, y, btn_w, btn_h)
            pygame.draw.rect(self.screen, LIGHT_BLUE, self.exit_rect)
            e_surf = self.font.render('Exit', True, BLACK)
            self.screen.blit(e_surf, (x + 20, y + 4))

            # Save button
            x2 = x - padding - btn_w
            self.save_as_rect = pygame.Rect(x2, y, btn_w, btn_h)
            pygame.draw.rect(self.screen, GREEN, self.save_as_rect)
            sa_surf = self.font.render('Save', True, BLACK)
            tw = sa_surf.get_width()
            self.screen.blit(sa_surf, (x2 + (btn_w - tw) // 2, y + 4))

            # Open button
            x3 = x2 - padding - btn_w
            self.open_rect = pygame.Rect(x3, y, btn_w, btn_h)
            pygame.draw.rect(self.screen, LIGHT_GRAY, self.open_rect)
            o_surf = self.font.render('Open', True, BLACK)
            self.screen.blit(o_surf, (x3 + 20, y + 4))

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

    def save(self, path: str = None):
        if path is None:
            path = self.scene_path
        # use editor start/goal if available
        if self.start_pose is not None:
            sx, sy, st = self.start_pose
            scene_start = {'x': sx, 'y': sy, 'theta': st}
        else:
            scene_start = None
        if self.goal_pose is not None:
            gx, gy, gt = self.goal_pose
            scene_goal = {'x': gx, 'y': gy, 'theta': gt}
        else:
            scene_goal = None
        scene = {'start': scene_start, 'goal': scene_goal, 'obstacles': []}
        # store obstacles
        for obs in self.obstacles:
            if isinstance(obs, CircleObstacle):
                scene['obstacles'].append({'type':'circle','x':obs.x,'y':obs.y,'radius':obs.radius})
            elif isinstance(obs, PolygonObstacle):
                scene['obstacles'].append({'type':'polygon','vertices':obs.vertices,'color':obs.color})
            elif isinstance(obs, LineObstacle):
                v = obs.vertices
                start = v[0]
                end = v[2]
                width = math.hypot(v[0][0]-v[1][0], v[0][1]-v[1][1])
                scene['obstacles'].append({'type':'line','start':start,'end':end,'width':width,'color':obs.color})
        with open(path, 'w') as fh:
            json.dump(scene, fh, indent=2)
        # update internal scene path
        self.scene_path = path

    def run(self):
        running = True
        # Ensure UI rects exist
        self.draw()
        while running:
            for event in pygame.event.get():
                # Always allow quitting even if a dialog is open
                if event.type == pygame.QUIT:
                    running = False
                    continue
                # If a modal dialog is open, skip processing other pygame events
                if self.dialog_open:
                    continue
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # left click
                        mx, my = event.pos
                        # embedded top-right interactions: Return | Save | Set Start | Set Goal
                        if self.embedded and hasattr(self, 'return_rect') and self.return_rect.collidepoint(mx, my):
                            # Return to parent (caller handles toggling)
                            running = False
                            continue
                        if self.embedded and hasattr(self, 'save_rect') and self.save_rect.collidepoint(mx, my):
                            if not self.save_text:
                                # First click: Show text input
                                self.save_text_active = True
                                self.save_text = ""  # Clear any previous text
                            elif self.save_text and self.save_text_active:
                                # Second click with text: Trigger save
                                map_path = os.path.join("maps", self.save_text + ".json")
                                if self.save_map(map_path, self.start_pose, self.goal_pose):
                                    self.save_text_active = False  # Hide input after successful save
                            continue
                        # Tool/Start/Goal buttons in bottom toolbox
                        for name, rect in list(self.toolbox_rects.items()):
                            if rect.collidepoint(mx, my):
                                # Map buttons to internal behaviors
                                if name in ('move','select','circle','line','polygon'):
                                    self.selected_tool = name
                                elif name == 'set_start':
                                    self.placing_start = True
                                    self.placing_goal = False
                                elif name == 'set_goal':
                                    self.placing_goal = True
                                    self.placing_start = False
                                # stop processing other logic
                                break
                        else:
                            # if no button matched, treat as map click below
                            pass
                        # top-right interactions: Open | Save As | Exit (standalone editor)
                        if hasattr(self, 'open_rect') and self.open_rect.collidepoint(mx, my):
                            # open file dialog
                            if filedialog is not None:
                                try:
                                    # fully shutdown the pygame display so the OS dialog can take focus
                                    try:
                                        pygame.display.quit()
                                    except Exception:
                                        pass
                                    pygame.event.clear()
                                    self.dialog_open = True
                                    root = tk.Tk(); root.withdraw(); root.attributes('-topmost', True)
                                    path = filedialog.askopenfilename(title='Open scene', filetypes=[('JSON','*.json')])
                                finally:
                                    try:
                                        root.destroy()
                                    except Exception:
                                        pass
                                    self.dialog_open = False
                                    # re-init pygame display and recreate screen/font
                                    try:
                                        pygame.display.init()
                                        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
                                        self.font = pygame.font.SysFont('monospace', 18)
                                    except Exception:
                                        pass
                                    pygame.event.clear()
                            else:
                                path = None
                            if path:
                                try:
                                    loaded = load_obstacles_from_json(path)
                                    self.obstacles = loaded
                                    self.scene_path = path
                                except Exception as e:
                                    print(f"Failed to open scene: {e}")
                        elif hasattr(self, 'save_as_rect') and self.save_as_rect.collidepoint(mx, my):
                            # save as dialog
                            if filedialog is not None:
                                try:
                                    try:
                                        pygame.display.quit()
                                    except Exception:
                                        pass
                                    pygame.event.clear()
                                    self.dialog_open = True
                                    root = tk.Tk(); root.withdraw(); root.attributes('-topmost', True)
                                    save_path = filedialog.asksaveasfilename(defaultextension='.json', filetypes=[('JSON','*.json')], title='Save scene as')
                                finally:
                                    try:
                                        root.destroy()
                                    except Exception:
                                        pass
                                    self.dialog_open = False
                                    try:
                                        pygame.display.init()
                                        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
                                        self.font = pygame.font.SysFont('monospace', 18)
                                    except Exception:
                                        pass
                                    pygame.event.clear()
                            else:
                                save_path = None
                            if save_path:
                                try:
                                    self.save(save_path)
                                except Exception as e:
                                    print(f"Failed to save scene: {e}")
                        elif hasattr(self, 'exit_rect') and self.exit_rect.collidepoint(mx, my):
                            running = False
                        else:
                            # check toolbox clicks
                            clicked_tool = None
                            for t, rect in self.toolbox_rects.items():
                                if rect.collidepoint(mx, my):
                                    clicked_tool = t
                                    break
                            if clicked_tool is not None:
                                self.selected_tool = clicked_tool
                            else:
                                wx, wy = self.screen_to_world(mx, my)
                                # Handle start/goal placement first
                                if self.placing_start:
                                    if self.start_pose is None or len(self.start_pose) < 4:
                                        # First click: store position and wait for direction
                                        self.start_pose = (wx, wy, 0.0, False)  # False means direction not set
                                    else:
                                        # Second click: finalize direction
                                        x, y, _, _ = self.start_pose
                                        theta = math.atan2(wy - y, wx - x)
                                        self.start_pose = (x, y, theta)
                                        self.placing_start = False
                                        self.selected_tool = 'select'
                                    continue
                                if self.placing_goal:
                                    if self.goal_pose is None or len(self.goal_pose) < 4:
                                        # First click: store position and wait for direction
                                        self.goal_pose = (wx, wy, 0.0, False)  # False means direction not set
                                    else:
                                        # Second click: finalize direction
                                        x, y, _, _ = self.goal_pose
                                        theta = math.atan2(wy - y, wx - x)
                                        self.goal_pose = (x, y, theta)
                                        self.placing_goal = False
                                        self.selected_tool = 'select'
                                    continue
                                
                                # Handle selection/dragging
                                if self.selected_tool == 'select':
                                    # Try start/goal first
                                    if self.start_pose is not None:
                                        sx, sy, st = self.start_pose
                                        if math.hypot(sx - wx, sy - wy) <= 0.5:
                                            self.selected_idx = 'start'
                                            self.dragging = True
                                            self.drag_last = (wx, wy)
                                            continue
                                    if self.goal_pose is not None:
                                        gx, gy, gt = self.goal_pose
                                        if math.hypot(gx - wx, gy - wy) <= 0.5:
                                            self.selected_idx = 'goal'
                                            self.dragging = True
                                            self.drag_last = (wx, wy)
                                            continue
                                    # Then try obstacles
                                    # if not already handling start/goal selection above, try obstacles
                                    idx = self.obstacle_at_point(wx, wy)
                                    self.selected_idx = idx
                                    if idx is not None:
                                        # start dragging obstacle
                                        self.dragging = True
                                        self.drag_last = (wx, wy)
                                elif self.selected_tool == 'circle':
                                    # place circle
                                    c = CircleObstacle(wx, wy, self.obstacle_size)
                                    self.obstacles.append(c)
                                elif self.selected_tool == 'polygon':
                                    self.temp_polygon.append((wx, wy))
                                elif self.selected_tool == 'line':
                                    if self.line_start is None:
                                        self.line_start = (wx, wy)
                                    else:
                                        line = LineObstacle(self.line_start, (wx, wy), self.obstacle_size)
                                        self.obstacles.append(line)
                                        self.line_start = None
                                elif self.selected_tool == 'move':
                                    # start camera pan with left button
                                    self.panning = True
                                    self.drag_last = event.pos
                    elif event.button in (2,3):
                        # middle or right = pan start
                        self.dragging = True
                        self.drag_last = event.pos
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button in (2,3):
                        self.dragging = False
                        self.drag_last = None
                    if event.button == 1:
                        self.dragging = False
                        self.drag_last = None
                        # stop panning if move tool was used
                        if getattr(self, 'panning', False):
                            self.panning = False
                            self.drag_last = None
                elif event.type == pygame.MOUSEMOTION:
                    if self.dragging and self.selected_tool == 'select' and self.selected_idx is not None and isinstance(self.selected_idx,int):
                        mx,my = event.pos
                        wx,wy = self.screen_to_world(mx,my)
                        obs = self.obstacles[self.selected_idx]
                        # for polygons and lines, move by translating vertices
                        if isinstance(obs, CircleObstacle):
                            obs.x = wx
                            obs.y = wy
                        elif isinstance(obs, PolygonObstacle):
                            cx = sum(v[0] for v in obs.vertices)/len(obs.vertices)
                            cy = sum(v[1] for v in obs.vertices)/len(obs.vertices)
                            dx = wx - cx
                            dy = wy - cy
                            obs.vertices = [(x+dx, y+dy) for x,y in obs.vertices]
                        self.drag_last = (mx,my)
                    elif self.dragging and self.selected_idx in ('start', 'goal'):
                        # Move the start/goal based on mouse
                        mx,my = event.pos
                        wx,wy = self.screen_to_world(mx,my)
                        if self.selected_idx == 'start':
                            heading = self.start_pose[2] if self.start_pose is not None else 0.0
                            self.start_pose = (wx, wy, heading)
                        else:
                            heading = self.goal_pose[2] if self.goal_pose is not None else 0.0
                            self.goal_pose = (wx, wy, heading)
                        self.drag_last = (mx,my)
                    elif getattr(self, 'panning', False) and isinstance(self.drag_last, tuple):
                        # camera pan using move tool or right/middle drag
                        mx,my = event.pos
                        lx,ly = self.drag_last
                        dx = (lx - mx) / SCALE
                        dy = (my - ly) / SCALE
                        self.camera_x += dx
                        self.camera_y += dy
                        self.drag_last = event.pos
                    elif self.dragging and isinstance(self.drag_last, tuple):
                        # pan map (right/middle button)
                        mx,my = event.pos
                        lx,ly = self.drag_last
                        dx = (lx - mx) / SCALE
                        dy = (my - ly) / SCALE
                        self.camera_x += dx
                        self.camera_y += dy
                        self.drag_last = event.pos
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_1:
                        self.selected_tool = 'move'
                    elif event.key == pygame.K_2:
                        self.selected_tool = 'select'
                    elif event.key == pygame.K_3:
                        self.selected_tool = 'circle'
                    elif event.key == pygame.K_4:
                        self.selected_tool = 'line'
                    elif event.key == pygame.K_5:
                        self.selected_tool = 'polygon'
                    elif event.key == pygame.K_RETURN and len(self.temp_polygon) >= 3:
                        # use distinct polygon color (green)
                        poly = PolygonObstacle(self.temp_polygon, (0,200,0))
                        self.obstacles.append(poly)
                        self.temp_polygon = []
                    # If embedded and save text active, Enter means notify parent to save (parent reads editor.save_text)
                    elif event.key == pygame.K_RETURN and self.embedded and self.save_text_active:
                        # nothing to do here; parent handles the actual save when it detects save_text_active and save_text
                        self.save_text_active = True
                        continue

            # simple UI hover/tool selection
            mx,my = pygame.mouse.get_pos()
            # handle clicking on toolbox (mouse clicks handle tool selection)

            # redraw UI each frame
            self.draw()
            self.clock.tick(FPS)

        # Only quit pygame if running standalone
        if not self.embedded:
            pygame.quit()

# Remove the if __name__ check as this isn't meant to be run standalone
