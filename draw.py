import pygame
import math
import numpy as np
from constants import *


class CubicPolynomial:
    """Least-squares cubic fit y ≈ a3 x^3 + a2 x^2 + a1 x + a0."""

    def __init__(self):
        self.a3 = 0.0
        self.a2 = 0.0
        self.a1 = 0.0
        self.a0 = 0.0
        self.valid = False

    def fit(self, points):
        """Fit coefficients from a list of (x, y) tuples."""
        if not points or len(points) < 4:
            self.valid = False
            return False

        rows = []
        values = []
        for x, y in points:
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            rows.append([x ** 3, x ** 2, x, 1.0])
            values.append(y)

        if len(rows) < 4:
            self.valid = False
            return False

        A = np.asarray(rows, dtype=float)
        b = np.asarray(values, dtype=float)
        try:
            coeffs, *_ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            self.valid = False
            return False

        self.a3, self.a2, self.a1, self.a0 = coeffs
        self.valid = True
        return True

    def at(self, x):
        """Evaluate polynomial at x."""
        return ((self.a3 * x + self.a2) * x + self.a1) * x + self.a0

    def deriv(self, x):
        """Derivative dy/dx at x."""
        return (3.0 * self.a3 * x + 2.0 * self.a2) * x + self.a1


def world_to_screen(x, y, camera_x, camera_y, zoom=1.0):
    sx = int((x - camera_x) * SCALE * zoom + SCREEN_WIDTH / 2)
    sy = int(-(y - camera_y) * SCALE * zoom + SCREEN_HEIGHT / 2)
    return sx, sy


def screen_to_world(sx, sy, camera_x, camera_y, zoom=1.0):
    wx = (sx - SCREEN_WIDTH/2) / (SCALE * zoom) + camera_x
    wy = -((sy - SCREEN_HEIGHT/2) / (SCALE * zoom)) + camera_y
    return wx, wy


def draw_start_goal(surface, start_pose, goal_pose, camera_x, camera_y, zoom=1.0):
    """Draw start and goal poses with orientation indicators."""
    if start_pose is not None:
        sx, sy, stheta = start_pose
        s_pos = world_to_screen(sx, sy, camera_x, camera_y, zoom)
        pygame.draw.circle(surface, START_COLOR, s_pos, 8)
        stx = sx + 0.75 * math.cos(stheta)
        sty = sy + 0.75 * math.sin(stheta)
        pygame.draw.line(surface, START_COLOR, s_pos, world_to_screen(stx, sty, camera_x, camera_y, zoom), 3)
    
    if goal_pose is not None:
        gx, gy, gtheta = goal_pose
        g_pos = world_to_screen(gx, gy, camera_x, camera_y, zoom)
        pygame.draw.circle(surface, GOAL_COLOR, g_pos, 8)
        gtx = gx + 0.75 * math.cos(gtheta)
        gty = gy + 0.75 * math.sin(gtheta)
        pygame.draw.line(surface, GOAL_COLOR, g_pos, world_to_screen(gtx, gty, camera_x, camera_y, zoom), 3)




def draw_grid(surface, camera_x, camera_y, grid_size=50, zoom=1.0):
    """Draw a grid centered on the camera position."""
    for i in range(-grid_size, grid_size + 1):
        start_pos = world_to_screen(i, -grid_size, camera_x, camera_y, zoom)
        end_pos = world_to_screen(i, grid_size, camera_x, camera_y, zoom)
        pygame.draw.line(surface, LIGHT_GRAY, start_pos, end_pos, 1)
        start_pos = world_to_screen(-grid_size, i, camera_x, camera_y, zoom)
        end_pos = world_to_screen(grid_size, i, camera_x, camera_y, zoom)
        pygame.draw.line(surface, LIGHT_GRAY, start_pos, end_pos, 1)


def draw_obstacles(surface, obstacles, camera_x, camera_y, zoom=1.0):
    """Draw all obstacles using their draw method."""
    if not obstacles:
        return
        
    def _w2s(x, y):
        return world_to_screen(x, y, camera_x, camera_y, zoom)
        
    for obs in obstacles:
        if hasattr(obs, 'draw'):
            obs.draw(surface, _w2s)


def draw_lane_centerline(surface, lane_points, vehicle, camera_x, camera_y, max_points=6, zoom=1.0):
    """Render lane centerline starting from the closest point in front of the vehicle."""
    if not lane_points:
        return

    start_idx = 0
    if vehicle is not None and lane_points:
        min_dist = float('inf')
        for idx, (x, y) in enumerate(lane_points):
            dx = x - vehicle.x
            dy = y - vehicle.y
            dist = dx * dx + dy * dy
            if dist < min_dist:
                min_dist = dist
                start_idx = idx

    end_idx = min(len(lane_points), start_idx + max_points)
    if start_idx >= end_idx:
        start_idx = max(0, len(lane_points) - max_points)
        end_idx = len(lane_points)

    subset = lane_points[start_idx:end_idx]
    for p in [world_to_screen(x, y, camera_x, camera_y, zoom) for x, y in subset]:
        try:
            pygame.draw.circle(surface, ORANGE, p, 4)
        except Exception:
            pass

    cubic = CubicPolynomial()
    if not cubic.fit(subset):
        return

    xs = [x for x, _ in subset if math.isfinite(x)]
    if len(xs) < 2:
        return

    min_x = min(xs)
    max_x = max(xs)
    if math.isclose(min_x, max_x, rel_tol=1e-6, abs_tol=1e-6):
        return

    span = max_x - min_x
    # Draw the polynomial beyond just the fitted samples so the full curve is visible.
    extend = max(span, 3.0)
    view_half_width = SCREEN_WIDTH / (2.0 * SCALE * zoom)
    draw_min = min(min_x - extend, camera_x - view_half_width - 2.0)
    draw_max = max(max_x + extend, camera_x + view_half_width + 2.0)

    render_span = max(draw_max - draw_min, 1e-6)
    step = max(render_span / 120.0, 0.05)
    samples = []
    t = draw_min
    while t <= draw_max + 1e-6:
        y = cubic.at(t)
        samples.append(world_to_screen(t, y, camera_x, camera_y, zoom))
        t += step
    samples.append(world_to_screen(draw_max, cubic.at(draw_max), camera_x, camera_y, zoom))

    if len(samples) >= 2:
        try:
            pygame.draw.lines(surface, ORANGE, False, samples, 2)
        except Exception:
            pass


def draw_toolbox(surface, editor):
    """Draw the editor toolbox and controls."""
    # Clear and recreate toolbox rectangles
    editor.toolbox_rects = {}

    # Draw bottom toolbox
    box_h = 140
    box_y = SCREEN_HEIGHT - box_h
    pygame.draw.rect(surface, BOX_BG, (0, box_y, SCREEN_WIDTH, box_h))
    
    # Tool buttons layout
    pairs = [
        ('move', 'select'),
        ('remove', None),
        ('circle', 'line'),
        ('polygon', 'waypoint'),
        ('start_marker', 'goal_marker')
    ]
    col_w = 160
    gap = 20
    cols = len(pairs)
    total_w = cols * col_w + (cols - 1) * gap
    start_x = (SCREEN_WIDTH - total_w) // 2
    top_y = box_y + 10
    bot_y = top_y + 46
    btn_h = 36
    
    # Button labels
    label_map = {'move': 'Move', 'select': 'Select', 'circle': 'Cones', 
                 'line': 'Lanes', 'polygon': 'Polygon', 'start_marker': 'Start Marker', 
                 'goal_marker': 'Goal Marker', 'waypoint': 'Waypoint', 'remove': 'Remove'}

    # Draw tool buttons
    for i, (top, bot) in enumerate(pairs):
        x = start_x + i * (col_w + gap)
        if top is not None:
            rect_top = pygame.Rect(x, top_y, col_w, btn_h)
            editor.toolbox_rects[top] = rect_top
            color = BTN_ACTIVE if editor.selected_tool == top else BTN_INACTIVE
            pygame.draw.rect(surface, color, rect_top)
            surf = editor.font.render(label_map.get(top, top), True, BLACK)
            surface.blit(surf, (x + 10, top_y + 6))
        if bot is not None:
            rect_bot = pygame.Rect(x, bot_y, col_w, btn_h)
            editor.toolbox_rects[bot] = rect_bot
            color = BTN_ACTIVE if editor.selected_tool == bot else BTN_INACTIVE
            pygame.draw.rect(surface, color, rect_bot)
            surf = editor.font.render(label_map.get(bot, bot), True, BLACK)
            surface.blit(surf, (x + 10, bot_y + 6))
            # No indicator for start/goal — user prefers no bulb indicator here

    # (removed obstacle size display per user preference)

    # Draw top-right controls
    if hasattr(editor, 'return_rect'):
        pygame.draw.rect(surface, EDIT_BTN, editor.return_rect)
        text = editor.font.render('Return', True, BLACK)
        surface.blit(text, (editor.return_rect.x + 10, editor.return_rect.y + 4))
    
    if hasattr(editor, 'save_rect'):
        save_color = GREEN if getattr(editor, 'save_text', '') else SAVE_DISABLED
        pygame.draw.rect(surface, save_color, editor.save_rect)
        text = editor.font.render('Save', True, BLACK)
        text_rect = text.get_rect(center=editor.save_rect.center)
        surface.blit(text, text_rect)
        
        if getattr(editor, 'save_text_active', False) and hasattr(editor, 'save_text_rect'):
            pygame.draw.rect(surface, WHITE, editor.save_text_rect)
            display_text = editor.save_text if editor.save_text else editor.save_text_placeholder
            text = editor.font.render(display_text, True, BLACK)
            text_rect = text.get_rect(midleft=(editor.save_text_rect.left + 10, editor.save_text_rect.centery))
            surface.blit(text, text_rect)
            # Draw blinking caret at end of entered text when active (only when actual editing)
            if getattr(editor, 'save_text_active', False):
                # use actual save_text (not placeholder) for caret position
                content = editor.save_text if editor.save_text else ''
                content_surf = editor.font.render(content, True, BLACK)
                caret_x = editor.save_text_rect.left + 10 + content_surf.get_width() + 1
                caret_y1 = editor.save_text_rect.top + 6
                caret_y2 = editor.save_text_rect.bottom - 6
                # Blink every 500ms
                if (pygame.time.get_ticks() // 500) % 2 == 0:
                    pygame.draw.line(surface, BLACK, (caret_x, caret_y1), (caret_x, caret_y2), 2)


def draw_map_selector(surface, sim):
    """Draw the map selector dropdown and edit button for the simulator."""
    if not getattr(sim, 'show_map_controls', True):
        return
    dropdown_rect = getattr(sim, 'dropdown_rect', pygame.Rect(SCREEN_WIDTH - 220, 10, 150, 30))
    edit_button_rect = getattr(sim, 'edit_button_rect', pygame.Rect(SCREEN_WIDTH - 60, 10, 50, 30))
    font = getattr(sim, 'font', pygame.font.SysFont('monospace', 18))
    # Draw dropdown button
    pygame.draw.rect(surface, DROPDOWN_BG, dropdown_rect)
    name = sim.current_map.replace('.json', '') if sim.current_map else 'Select Map'
    text = font.render(name[:18], True, BLACK)
    surface.blit(text, (dropdown_rect.x + 5, dropdown_rect.y + 5))

    # Draw dropdown list if open
    if getattr(sim, 'dropdown_open', False):
        y = dropdown_rect.bottom
        for map_name in (getattr(sim, 'available_maps', []) + ['New Map']):
            r = pygame.Rect(dropdown_rect.x, y, dropdown_rect.width, 30)
            pygame.draw.rect(surface, WHITE, r)
            pygame.draw.rect(surface, BORDER_GRAY, r, 1)
            display_name = map_name.replace('.json', '') if map_name != 'New Map' else map_name
            text = font.render(display_name[:18], True, BLACK)
            surface.blit(text, (r.x + 5, r.y + 5))
            y += 30

    # Draw edit button
    pygame.draw.rect(surface, EDIT_BTN, edit_button_rect)
    text = font.render('Edit', True, WHITE)
    text_rect = text.get_rect(center=edit_button_rect.center)
    surface.blit(text, text_rect)


def draw_wheel(surface, center_x, center_y, width, length, wheel_angle, camera_x, camera_y, zoom=1.0):
    """Helper to draw a single wheel given world coords and wheel angle."""
    half_length = length / 2
    half_width = width / 2
    corners = [
        (-half_length, -half_width),
        (-half_length, half_width),
        (half_length, half_width),
        (half_length, -half_width),
    ]
    world_corners = []
    for x_local, y_local in corners:
        x_world = center_x + x_local * math.cos(wheel_angle) - y_local * math.sin(wheel_angle)
        y_world = center_y + x_local * math.sin(wheel_angle) + y_local * math.cos(wheel_angle)
        world_corners.append((x_world, y_world))
    screen_corners = [world_to_screen(x, y, camera_x, camera_y, zoom) for x, y in world_corners]
    pygame.draw.polygon(surface, BLACK, screen_corners)


def draw_vehicle(surface, vehicle, camera_x, camera_y, is_colliding=False, zoom=1.0):
    """Draw the vehicle polygon, front indicator line, and wheels.

    vehicle: object with x,y,heading,length,width,wheelbase,track_width,steering_angle
    """
    # Vehicle polygon
    half_length = vehicle.length / 2
    half_width = vehicle.width / 2
    local_corners = [
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width)
    ]
    world_corners = []
    for x_local, y_local in local_corners:
        x_world = vehicle.x + x_local * math.cos(vehicle.heading) - y_local * math.sin(vehicle.heading)
        y_world = vehicle.y + x_local * math.sin(vehicle.heading) + y_local * math.cos(vehicle.heading)
        world_corners.append((x_world, y_world))
    screen_corners = [world_to_screen(x, y, camera_x, camera_y, zoom) for x, y in world_corners]
    vehicle_color = RED if is_colliding else LIGHT_BLUE
    pygame.draw.polygon(surface, vehicle_color, screen_corners)

    # Front indicator line
    front_mid_x = (world_corners[0][0] + world_corners[1][0]) / 2
    front_mid_y = (world_corners[0][1] + world_corners[1][1]) / 2
    center_screen = world_to_screen(vehicle.x, vehicle.y, camera_x, camera_y, zoom)
    front_screen = world_to_screen(front_mid_x, front_mid_y, camera_x, camera_y, zoom)
    pygame.draw.line(surface, YELLOW, center_screen, front_screen, 3)

    # Wheels
    wheel_width = 0.07
    wheel_length = 0.2
    half_wheelbase = vehicle.wheelbase / 2
    half_track = vehicle.track_width / 2
    # Back wheels (aligned with heading)
    for side in [-1, 1]:
        back_wheel_x = vehicle.x - half_wheelbase * math.cos(vehicle.heading) + side * half_track * math.cos(vehicle.heading + math.pi/2)
        back_wheel_y = vehicle.y - half_wheelbase * math.sin(vehicle.heading) + side * half_track * math.sin(vehicle.heading + math.pi/2)
        draw_wheel(surface, back_wheel_x, back_wheel_y, wheel_width, wheel_length, vehicle.heading, camera_x, camera_y, zoom)

    # Front wheels (steering)
    right_wheel_angle = math.atan((vehicle.wheelbase*math.tan(vehicle.steering_angle))/(vehicle.wheelbase + (vehicle.track_width/2)*math.tan(vehicle.steering_angle)))
    left_wheel_angle = math.atan((vehicle.wheelbase*math.tan(vehicle.steering_angle))/(vehicle.wheelbase - (vehicle.track_width/2)*math.tan(vehicle.steering_angle)))
    for side, wheel_angle in zip([-1, 1], [right_wheel_angle, left_wheel_angle]):
        front_wheel_x = vehicle.x + half_wheelbase * math.cos(vehicle.heading) + side * half_track * math.cos(vehicle.heading + math.pi/2)
        front_wheel_y = vehicle.y + half_wheelbase * math.sin(vehicle.heading) + side * half_track * math.sin(vehicle.heading + math.pi/2)
        draw_wheel(surface, front_wheel_x, front_wheel_y, wheel_width, wheel_length, vehicle.heading + wheel_angle, camera_x, camera_y, zoom)


def draw_hud(surface, vehicle, font, follow_planner, use_pure_pursuit=False, planner_mode=None, evaluator=None, perception_mode=False, debug_mode=False):
    """Displays vehicle state information on the screen (HUD)."""
    try:
        speed_kmh = vehicle.speed * 3.6
        steer_deg = math.degrees(vehicle.steering_angle)

        info = [
            f"Speed: {speed_kmh:.1f} km/h",
            f"Steering: {steer_deg:.1f} degrees",
            f"Position: ({vehicle.x:.1f}, {vehicle.y:.1f}) m",
            f"Heading: {math.degrees(vehicle.heading):.1f} degrees"
        ]

        for i, line in enumerate(info):
            text_surface = font.render(line, True, WHITE)
            surface.blit(text_surface, (10, 10 + i * 25))

        follow_text = f"Follow planner (SPACE): {'ON' if follow_planner else 'OFF'}"
        follow_surface = font.render(follow_text, True, GREEN if follow_planner else FOLLOW_OFF)
        surface.blit(follow_surface, (10, 10 + len(info) * 25))

        pp_text = f"Pure Pursuit (U): {'ON' if use_pure_pursuit else 'OFF'}"
        pp_surface = font.render(pp_text, True, LIGHT_BLUE if use_pure_pursuit else FOLLOW_OFF)
        surface.blit(pp_surface, (10, 10 + (len(info) + 1) * 25))

        pm_text = f"Perception/FOW (V): {'ON' if perception_mode else 'OFF'}"
        pm_surface = font.render(pm_text, True, (200, 200, 50) if perception_mode else FOLLOW_OFF)
        surface.blit(pm_surface, (10, 10 + (len(info) + 2) * 25))

        dbg_text = f"Debug heatmap (D): {'ON — click for cost' if debug_mode else 'OFF'}"
        dbg_surface = font.render(dbg_text, True, (255, 200, 0) if debug_mode else FOLLOW_OFF)
        surface.blit(dbg_surface, (10, 10 + (len(info) + 3) * 25))

        # --- Evaluator score (top-right) ---
        if evaluator is not None:
            try:
                ev = evaluator
                elapsed = ev.elapsed
                score_text = f"EVAL: {ev.score:+.0f}  t={elapsed:.0f}s  col={len(ev.collision_events)}  wp={ev.waypoints_reached}"
                ev_color = (0, 220, 0) if ev.score >= 0 else (255, 80, 80)
                if ev.finished:
                    ev_color = (255, 255, 0)
                    score_text = "EVAL DONE: " + score_text[6:]
                ev_surf = font.render(score_text, True, ev_color)
                ev_rect = ev_surf.get_rect()
                ev_rect.topright = (SCREEN_WIDTH - 10, 50)
                bg = ev_rect.inflate(10, 6)
                pygame.draw.rect(surface, (20, 20, 20), bg, border_radius=3)
                surface.blit(ev_surf, ev_rect)
            except Exception:
                pass

        # --- Mode indicator: bottom-centre ---
        raw_mode = planner_mode or 'GPS'
        parts = raw_mode.split(':')
        mode = parts[0]   # 'LANE' or 'GPS'
        wp_idx = int(parts[1]) if len(parts) > 1 else -1
        is_lane = (mode == 'LANE')
        if is_lane:
            mode_text = 'LANE MODE'
        elif wp_idx >= 0:
            mode_text = f'GPS MODE  \u2192  WP {wp_idx}'
        else:
            mode_text = 'GPS MODE'
        mode_color = (0, 220, 0) if is_lane else (220, 180, 0)
        try:
            big_font = pygame.font.SysFont('monospace', 22, bold=True)
        except Exception:
            big_font = font
        mode_surf = big_font.render(mode_text, True, mode_color)
        mode_rect = mode_surf.get_rect()
        mode_rect.midbottom = (SCREEN_WIDTH // 2, SCREEN_HEIGHT - 8)
        bg_rect = mode_rect.inflate(20, 10)
        pygame.draw.rect(surface, (20, 20, 20), bg_rect, border_radius=5)
        surface.blit(mode_surf, mode_rect)
    except Exception:
        pass


def draw_detected_lane_cl(surface, sim):
    """Draw the detected lane centerline (from scan_lane) as cyan dots."""
    try:
        pts = getattr(sim.pose_publisher, 'detected_lane_pts', [])
        if not pts:
            return
        cam_x = getattr(sim, 'camera_x', 0.0)
        cam_y = getattr(sim, 'camera_y', 0.0)
        zoom  = getattr(sim, 'camera_zoom', 1.0)
        for i, (wx, wy) in enumerate(pts):
            sx, sy = world_to_screen(wx, wy, cam_x, cam_y, zoom)
            r = 5 if i in (3, 8) else 3  # highlight the two MPC target indices
            color = (0, 255, 255) if i not in (3, 8) else (255, 200, 0)
            pygame.draw.circle(surface, color, (sx, sy), r)
            if i > 0:
                px, py = pts[i - 1]
                sxp, syp = world_to_screen(px, py, cam_x, cam_y, zoom)
                pygame.draw.line(surface, (0, 200, 200), (sxp, syp), (sx, sy), 1)
    except Exception:
        pass


def _get_cost_arr(sim):
    """Return the planner's Gaussian cost field (received via /cost_field topic)."""
    pub = sim.pose_publisher
    return getattr(pub, 'last_cost_arr', None)


def draw_debug_heatmap(surface, sim):
    """Render Gaussian cost field as colour heatmap overlay.
    Prefers the planner's cost field (from /cost_field topic), falls back to
    local EDT + Gaussian on the raw occupancy grid."""
    try:
        arr = _get_cost_arr(sim)
        if arr is None:
            return
        pub = sim.pose_publisher
        ox, oy = pub.last_grid_origin
        res     = pub.last_grid_res
        cam_x   = sim.camera_x
        cam_y   = sim.camera_y
        zoom    = sim.camera_zoom

        nx, ny = arr.shape
        cell_px = max(1, int(res * SCALE * zoom))

        for ix in range(nx):
            for iy in range(ny):
                v = arr[ix, iy]
                if v <= 0:
                    continue
                # World centre of this cell
                wx = ox + (ix + 0.5) * res
                wy = oy + (iy + 0.5) * res
                sx, sy = world_to_screen(wx, wy, cam_x, cam_y, zoom)
                # Colour: 0→100 maps green→red
                t = min(v / 100.0, 1.0)
                r = int(50 + 205 * t)
                g = int(200 * (1.0 - t))
                b = 40
                alpha = 180
                cell_surf = pygame.Surface((cell_px, cell_px), pygame.SRCALPHA)
                cell_surf.fill((r, g, b, alpha))
                surface.blit(cell_surf, (sx - cell_px // 2, sy - cell_px // 2))
    except Exception:
        pass


def draw_debug_cost_label(surface, font, wx, wy, sim):
    """Print the cost field value at a clicked world position."""
    try:
        pub = sim.pose_publisher
        arr = _get_cost_arr(sim)
        if arr is None:
            return
        ox, oy = pub.last_grid_origin
        res     = pub.last_grid_res
        ix = int((wx - ox) / res)
        iy = int((wy - oy) / res)
        nx, ny = arr.shape
        if 0 <= ix < nx and 0 <= iy < ny:
            cost = int(arr[ix, iy])
            sx, sy = world_to_screen(wx, wy, sim.camera_x, sim.camera_y, sim.camera_zoom)
            label = font.render(f"cost={cost} ({wx:.1f},{wy:.1f})", True, (255, 255, 0))
            surface.blit(label, (sx + 6, sy - 10))
    except Exception:
        pass


def draw_cumulative_lane_trail(surface, sim):
    """Draw the cumulative lane centerline trail (all detected positions over time)
    as a persistent magenta/pink trail on the map. This lets the user see everywhere
    the lane detector has found a centerline."""
    try:
        trail = getattr(sim.pose_publisher, 'cumulative_lane_trail', [])
        if not trail:
            return
        cam_x = getattr(sim, 'camera_x', 0.0)
        cam_y = getattr(sim, 'camera_y', 0.0)
        zoom  = getattr(sim, 'camera_zoom', 1.0)
        # Draw trail as connected line segments
        screen_pts = [world_to_screen(wx, wy, cam_x, cam_y, zoom) for wx, wy in trail]
        if len(screen_pts) >= 2:
            pygame.draw.lines(surface, (220, 80, 220), False, screen_pts, 2)
        # Draw dots at each trail point
        for sp in screen_pts:
            pygame.draw.circle(surface, (200, 60, 200), sp, 2)
    except Exception:
        pass


def draw_mpc_targets(surface, sim):
    """Draw the MPC planned trajectory waypoints as distinct markers.
    These are the actual positions the MPC optimizer chose."""
    try:
        pts = getattr(sim.pose_publisher, 'mpc_target_pts', [])
        if not pts:
            return
        cam_x = getattr(sim, 'camera_x', 0.0)
        cam_y = getattr(sim, 'camera_y', 0.0)
        zoom  = getattr(sim, 'camera_zoom', 1.0)
        screen_pts = [world_to_screen(wx, wy, cam_x, cam_y, zoom) for wx, wy in pts]
        # Draw path line
        if len(screen_pts) >= 2:
            pygame.draw.lines(surface, (255, 120, 0), False, screen_pts, 2)
        # Draw each waypoint as an orange diamond
        for i, (sx, sy) in enumerate(screen_pts):
            r = 4
            diamond = [(sx, sy - r), (sx + r, sy), (sx, sy + r), (sx - r, sy)]
            pygame.draw.polygon(surface, (255, 140, 0), diamond)
            if i == 0:
                # First point gets a bigger ring to show MPC start
                pygame.draw.circle(surface, (255, 100, 0), (sx, sy), 6, 2)
    except Exception:
        pass

def draw_lane_boundaries(surface, sim):
    """Draw the published lane boundary points (left=blue, right=red),
    the midpoint centerline (purple), and, in debug mode only, the triangular
    detection frustum."""
    try:
        cam_x = getattr(sim, 'camera_x', 0.0)
        cam_y = getattr(sim, 'camera_y', 0.0)
        zoom  = getattr(sim, 'camera_zoom', 1.0)
        vx    = getattr(sim.vehicle, 'x', 0.0)
        vy    = getattr(sim.vehicle, 'y', 0.0)
        vh    = getattr(sim.vehicle, 'heading', 0.0)

        if getattr(sim, 'debug_mode', False):
            # Draw triangular detection frustum only in debug mode.
            # Increased depth from 2m to 4m, preserving the 150-degree tip angle.
            DETECT_FWD = 4.0
            DETECT_LAT = math.tan(math.radians(75.0)) * DETECT_FWD
            cos_h, sin_h = math.cos(vh), math.sin(vh)
            # Triangle vertices in world coords: car position + two far corners
            tip   = (vx, vy)
            left_far  = (vx + cos_h * DETECT_FWD - sin_h * DETECT_LAT,
                         vy + sin_h * DETECT_FWD + cos_h * DETECT_LAT)
            right_far = (vx + cos_h * DETECT_FWD + sin_h * DETECT_LAT,
                         vy + sin_h * DETECT_FWD - cos_h * DETECT_LAT)
            pts = [
                world_to_screen(*tip,       cam_x, cam_y, zoom),
                world_to_screen(*left_far,  cam_x, cam_y, zoom),
                world_to_screen(*right_far, cam_x, cam_y, zoom),
            ]
            pygame.draw.polygon(surface, (255, 220, 0), pts, 2)

        # Left boundary (blue dots)
        left = getattr(sim.pose_publisher, 'last_lane_left_pts', [])
        for x, y in left:
            sp = world_to_screen(x, y, cam_x, cam_y, zoom)
            pygame.draw.circle(surface, (80, 120, 255), sp, 4)

        # Right boundary (red dots)
        right = getattr(sim.pose_publisher, 'last_lane_right_pts', [])
        for x, y in right:
            sp = world_to_screen(x, y, cam_x, cam_y, zoom)
            pygame.draw.circle(surface, (255, 80, 80), sp, 4)

        # Purple centerline dots — only when both sides are visible this frame
        # and the paired points are at least 1.5m apart (so we're actually between two lanes).
        MIN_LANE_SEP = 1.5
        if left and right:
            n = min(len(left), len(right))
            for i in range(n):
                sep = math.hypot(left[i][0] - right[i][0], left[i][1] - right[i][1])
                if sep < MIN_LANE_SEP:
                    continue
                mx = (left[i][0] + right[i][0]) / 2.0
                my = (left[i][1] + right[i][1]) / 2.0
                sp = world_to_screen(mx, my, cam_x, cam_y, zoom)
                pygame.draw.circle(surface, (180, 60, 220), sp, 5)

    except Exception:
        pass


def draw_permanent_centerline(surface, center_pts, camera_x, camera_y, zoom=1.0):
    """Draw persistently accumulated lane centerline points as purple dots.
    Only points recorded when both lane boundaries were simultaneously visible."""
    if not center_pts:
        return
    try:
        for x, y in center_pts:
            sp = world_to_screen(x, y, camera_x, camera_y, zoom)
            pygame.draw.circle(surface, (180, 60, 220), sp, 4)
    except Exception:
        pass


def draw_permanent_lanes(surface, left_pts, right_pts, camera_x, camera_y, zoom=1.0):
    """Draw permanently accumulated lane boundary points as dots only (no connecting lines).
    Left = blue, Right = red."""
    def _draw_side(pts, color):
        if not pts:
            return
        for x, y in pts:
            s = world_to_screen(x, y, camera_x, camera_y, zoom)
            pygame.draw.circle(surface, color, s, 3)

    try:
        _draw_side(left_pts, (60, 100, 255))
        _draw_side(right_pts, (255, 60, 60))
    except Exception:
        pass


def draw_planner_trajectory(surface, sim):
    """Draw the latest planner trajectory (or active trajectory) as a yellow polyline with waypoint markers.

    `sim` is the simulator instance; we access its pose_publisher/latest_trajectory_msg and camera
    position to transform waypoints into screen coordinates.
    """
    try:
        traj_msg = getattr(sim.pose_publisher, 'latest_trajectory_msg', None)
    except Exception:
        traj_msg = None

    if traj_msg is None:
        return

    points = []
    try:
        for wp in traj_msg.waypoints:
            points.append(world_to_screen(wp.x, wp.y, sim.camera_x, sim.camera_y, sim.camera_zoom))
    except Exception:
        return

    if len(points) == 0:
        return

    try:
        pygame.draw.lines(surface, YELLOW, False, points, 2)
    except Exception:
        pass

    for p in points:
        try:
            pygame.draw.circle(surface, YELLOW, p, 4)
        except Exception:
            pass

def draw_plotter_visible_grid(surface, vehicle, camera_x, camera_y, grid_width_m=30.0, grid_height_m=30.0, resolution=0.2, zoom=1.0):
    """Draw the visible grid window that the plotter sees (moving box).
    
    Shows the local area that occupancy grid computation considers.
    This is the window the waypoint plotter is working within.
    
    Args:
        grid_width_m: Width of grid in meters (center on vehicle)
        grid_height_m: Height of grid in meters (center on vehicle)
        resolution: Grid cell resolution in meters
    """
    if not vehicle:
        return
    
    # Grid bounds (vehicle-centered)
    half_width = grid_width_m / 2
    half_height = grid_height_m / 2
    
    # World coordinates
    left = vehicle.x - half_width
    right = vehicle.x + half_width
    bottom = vehicle.y - half_height
    top = vehicle.y + half_height
    
    # Convert to screen coordinates
    top_left = world_to_screen(left, top, camera_x, camera_y, zoom)
    top_right = world_to_screen(right, top, camera_x, camera_y, zoom)
    bottom_right = world_to_screen(right, bottom, camera_x, camera_y, zoom)
    bottom_left = world_to_screen(left, bottom, camera_x, camera_y, zoom)
    
    # Draw semi-transparent box (grid visibility area)
    pygame.draw.line(surface, (100, 200, 100), top_left, top_right, 2)  # Green top
    pygame.draw.line(surface, (100, 200, 100), top_right, bottom_right, 2)  # Green right
    pygame.draw.line(surface, (100, 200, 100), bottom_right, bottom_left, 2)  # Green bottom
    pygame.draw.line(surface, (100, 200, 100), bottom_left, top_left, 2)  # Green left
    
    # Draw grid lines inside the visible area (0.2m resolution)
    step_cells = max(1, int(resolution / 0.2))  # Draw every N cells
    
    # Vertical lines
    x = left
    while x <= right:
        screen_x_y_top = world_to_screen(x, top, camera_x, camera_y, zoom)
        screen_x_y_bot = world_to_screen(x, bottom, camera_x, camera_y, zoom)
        pygame.draw.line(surface, (80, 150, 80), screen_x_y_top, screen_x_y_bot, 1)
        x += resolution * 5  # Draw every 5th cell for readability
    
    # Horizontal lines
    y = bottom
    while y <= top:
        screen_x_left_y = world_to_screen(left, y, camera_x, camera_y, zoom)
        screen_x_right_y = world_to_screen(right, y, camera_x, camera_y, zoom)
        pygame.draw.line(surface, (80, 150, 80), screen_x_left_y, screen_x_right_y, 1)
        y += resolution * 5  # Draw every 5th cell for readability


def draw_rl_waypoints(surface, rl_waypoints, rl_selected_gap, vehicle, camera_x, camera_y, zoom=1.0):
    """Draw RL gap selector waypoints and selected gap direction.
    In RL mode: draw gap direction line + waypoints
    In lane mode: draw waypoints only (no line from vehicle)
    
    Args:
        surface: Pygame surface
        rl_waypoints: List of (x, y) intermediate waypoints
        rl_selected_gap: Selected gap dict with 'angle' and 'distance' keys (None for lane mode)
        vehicle: Vehicle object
        camera_x, camera_y: Camera position
        zoom: Camera zoom factor
    """
    if not rl_waypoints:
        return
    
    try:
        # Draw gap direction line if a gap is selected (RL mode only)
        if rl_selected_gap:
            gap_distance = rl_selected_gap.get('distance', 5.0)
            gap_angle = rl_selected_gap.get('angle', 0.0)
            
            end_x = vehicle.x + gap_distance * math.cos(gap_angle)
            end_y = vehicle.y + gap_distance * math.sin(gap_angle)
            
            vehicle_screen = world_to_screen(vehicle.x, vehicle.y, camera_x, camera_y, zoom)
            gap_end_screen = world_to_screen(end_x, end_y, camera_x, camera_y, zoom)
            
            # Draw gap direction as cyan line
            pygame.draw.line(surface, (0, 255, 255), vehicle_screen, gap_end_screen, 2)
        
        # Draw waypoints as yellow breadcrumb dots
        for wp_x, wp_y in rl_waypoints:
            wp_screen = world_to_screen(wp_x, wp_y, camera_x, camera_y, zoom)
            pygame.draw.circle(surface, YELLOW, wp_screen, 4)
            
    except Exception:
        pass