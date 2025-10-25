import pygame
import math
from constants import *


def world_to_screen(x, y, camera_x, camera_y):
    sx = int((x - camera_x) * SCALE + SCREEN_WIDTH / 2)
    sy = int(-(y - camera_y) * SCALE + SCREEN_HEIGHT / 2)
    return sx, sy


def screen_to_world(sx, sy, camera_x, camera_y):
    wx = (sx - SCREEN_WIDTH/2) / SCALE + camera_x
    wy = -((sy - SCREEN_HEIGHT/2) / SCALE) + camera_y
    return wx, wy


def draw_start_goal(surface, start_pose, goal_pose, camera_x, camera_y):
    """Draw start and goal poses with orientation indicators."""
    if start_pose is not None:
        sx, sy, stheta = start_pose
        s_pos = world_to_screen(sx, sy, camera_x, camera_y)
        pygame.draw.circle(surface, START_COLOR, s_pos, 8)
        stx = sx + 0.75 * math.cos(stheta)
        sty = sy + 0.75 * math.sin(stheta)
        pygame.draw.line(surface, START_COLOR, s_pos, world_to_screen(stx, sty, camera_x, camera_y), 3)
    
    if goal_pose is not None:
        gx, gy, gtheta = goal_pose
        g_pos = world_to_screen(gx, gy, camera_x, camera_y)
        pygame.draw.circle(surface, GOAL_COLOR, g_pos, 8)
        gtx = gx + 0.75 * math.cos(gtheta)
        gty = gy + 0.75 * math.sin(gtheta)
        pygame.draw.line(surface, GOAL_COLOR, g_pos, world_to_screen(gtx, gty, camera_x, camera_y), 3)


def draw_grid(surface, camera_x, camera_y, grid_size=50):
    """Draw a grid centered on the camera position."""
    for i in range(-grid_size, grid_size + 1):
        start_pos = world_to_screen(i, -grid_size, camera_x, camera_y)
        end_pos = world_to_screen(i, grid_size, camera_x, camera_y)
        pygame.draw.line(surface, LIGHT_GRAY, start_pos, end_pos, 1)
        start_pos = world_to_screen(-grid_size, i, camera_x, camera_y)
        end_pos = world_to_screen(grid_size, i, camera_x, camera_y)
        pygame.draw.line(surface, LIGHT_GRAY, start_pos, end_pos, 1)


def draw_obstacles(surface, obstacles, camera_x, camera_y):
    """Draw all obstacles using their draw method."""
    if not obstacles:
        return
        
    def _w2s(x, y):
        return world_to_screen(x, y, camera_x, camera_y)
        
    for obs in obstacles:
        if hasattr(obs, 'draw'):
            obs.draw(surface, _w2s)


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
        ('circle', 'line'),
        ('polygon', None),
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
                 'goal_marker': 'Goal Marker'}

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


def draw_wheel(surface, center_x, center_y, width, length, wheel_angle, camera_x, camera_y):
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
    screen_corners = [world_to_screen(x, y, camera_x, camera_y) for x, y in world_corners]
    pygame.draw.polygon(surface, BLACK, screen_corners)


def draw_vehicle(surface, vehicle, camera_x, camera_y, is_colliding=False):
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
    screen_corners = [world_to_screen(x, y, camera_x, camera_y) for x, y in world_corners]
    vehicle_color = RED if is_colliding else LIGHT_BLUE
    pygame.draw.polygon(surface, vehicle_color, screen_corners)

    # Front indicator line
    front_mid_x = (world_corners[0][0] + world_corners[1][0]) / 2
    front_mid_y = (world_corners[0][1] + world_corners[1][1]) / 2
    center_screen = world_to_screen(vehicle.x, vehicle.y, camera_x, camera_y)
    front_screen = world_to_screen(front_mid_x, front_mid_y, camera_x, camera_y)
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
        draw_wheel(surface, back_wheel_x, back_wheel_y, wheel_width, wheel_length, vehicle.heading, camera_x, camera_y)

    # Front wheels (steering)
    right_wheel_angle = math.atan((vehicle.wheelbase*math.tan(vehicle.steering_angle))/(vehicle.wheelbase + (vehicle.track_width/2)*math.tan(vehicle.steering_angle)))
    left_wheel_angle = math.atan((vehicle.wheelbase*math.tan(vehicle.steering_angle))/(vehicle.wheelbase - (vehicle.track_width/2)*math.tan(vehicle.steering_angle)))
    for side, wheel_angle in zip([-1, 1], [right_wheel_angle, left_wheel_angle]):
        front_wheel_x = vehicle.x + half_wheelbase * math.cos(vehicle.heading) + side * half_track * math.cos(vehicle.heading + math.pi/2)
        front_wheel_y = vehicle.y + half_wheelbase * math.sin(vehicle.heading) + side * half_track * math.sin(vehicle.heading + math.pi/2)
        draw_wheel(surface, front_wheel_x, front_wheel_y, wheel_width, wheel_length, vehicle.heading + wheel_angle, camera_x, camera_y)


def draw_hud(surface, vehicle, font, follow_planner):
    """Displays vehicle state information on the screen (HUD).

    Kept here so all rendering is centralized in draw.py. Parameters are the minimal
    primitives required for rendering: the surface, the vehicle object, a pygame
    Font object, and whether the planner-follow flag is set.
    """
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

            follow_text = f"Follow planner: {'ON' if follow_planner else 'OFF'}"
            follow_surface = font.render(follow_text, True, GREEN if follow_planner else FOLLOW_OFF)
            surface.blit(follow_surface, (10, 10 + len(info) * 25))
    except Exception:
        # Rendering should never raise for HUD; swallow errors
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
            points.append(world_to_screen(wp.x, wp.y, sim.camera_x, sim.camera_y))
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
