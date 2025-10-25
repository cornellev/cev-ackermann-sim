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
        pygame.draw.circle(surface, (160, 32, 240), s_pos, 8)
        stx = sx + 0.75 * math.cos(stheta)
        sty = sy + 0.75 * math.sin(stheta)
        pygame.draw.line(surface, (160, 32, 240), s_pos, world_to_screen(stx, sty, camera_x, camera_y), 3)
    
    if goal_pose is not None:
        gx, gy, gtheta = goal_pose
        g_pos = world_to_screen(gx, gy, camera_x, camera_y)
        pygame.draw.circle(surface, (0, 200, 0), g_pos, 8)
        gtx = gx + 0.75 * math.cos(gtheta)
        gty = gy + 0.75 * math.sin(gtheta)
        pygame.draw.line(surface, (0, 200, 0), g_pos, world_to_screen(gtx, gty, camera_x, camera_y), 3)


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
    pygame.draw.rect(surface, (40, 40, 40), (0, box_y, SCREEN_WIDTH, box_h))
    
    # Tool buttons layout
    pairs = [
        ('move', 'select'),
        ('circle', 'line'),
        ('polygon', None),
        ('set_start', 'set_goal')
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
                 'line': 'Lanes', 'polygon': 'Polygon', 'set_start': 'Start', 
                 'set_goal': 'Goal'}

    # Draw tool buttons
    for i, (top, bot) in enumerate(pairs):
        x = start_x + i * (col_w + gap)
        if top is not None:
            rect_top = pygame.Rect(x, top_y, col_w, btn_h)
            editor.toolbox_rects[top] = rect_top
            color = (200, 200, 200) if editor.selected_tool == top else (120, 120, 120)
            pygame.draw.rect(surface, color, rect_top)
            surf = editor.font.render(label_map.get(top, top), True, BLACK)
            surface.blit(surf, (x + 10, top_y + 6))
        if bot is not None:
            rect_bot = pygame.Rect(x, bot_y, col_w, btn_h)
            editor.toolbox_rects[bot] = rect_bot
            color = (200, 200, 200) if editor.selected_tool == bot else (120, 120, 120)
            pygame.draw.rect(surface, color, rect_bot)
            surf = editor.font.render(label_map.get(bot, bot), True, BLACK)
            surface.blit(surf, (x + 10, bot_y + 6))
            if bot == 'set_start':
                pygame.draw.circle(surface, (160, 32, 240), (rect_bot.right - 16, rect_bot.centery), 8)
            elif bot == 'set_goal':
                pygame.draw.circle(surface, (0, 200, 0), (rect_bot.right - 16, rect_bot.centery), 8)

    # Size display
    size_label = editor.font.render('Size:', True, WHITE)
    surface.blit(size_label, (start_x + total_w + 30, box_y + 14))
    size_text = editor.font.render(f"{editor.obstacle_size:.2f}m", True, WHITE)
    surface.blit(size_text, (start_x + total_w + 30, box_y + 40))

    # Draw top-right controls
    if hasattr(editor, 'return_rect'):
        pygame.draw.rect(surface, LIGHT_BLUE, editor.return_rect)
        text = editor.font.render('Return', True, BLACK)
        surface.blit(text, (editor.return_rect.x + 10, editor.return_rect.y + 4))
    
    if hasattr(editor, 'save_rect'):
        save_color = LIGHT_BLUE if getattr(editor, 'save_text', '') else (150, 150, 150)
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


def draw_map_selector(surface, sim):
    """Draw the map selector dropdown and edit button for the simulator."""
    if not getattr(sim, 'show_map_controls', True):
        return
    dropdown_rect = getattr(sim, 'dropdown_rect', pygame.Rect(SCREEN_WIDTH - 220, 10, 150, 30))
    edit_button_rect = getattr(sim, 'edit_button_rect', pygame.Rect(SCREEN_WIDTH - 60, 10, 50, 30))
    font = getattr(sim, 'font', pygame.font.SysFont('monospace', 18))
    # Draw dropdown button
    pygame.draw.rect(surface, (200, 200, 200), dropdown_rect)
    name = sim.current_map.replace('.json', '') if sim.current_map else 'Select Map'
    text = font.render(name[:18], True, (0, 0, 0))
    surface.blit(text, (dropdown_rect.x + 5, dropdown_rect.y + 5))

    # Draw dropdown list if open
    if getattr(sim, 'dropdown_open', False):
        y = dropdown_rect.bottom
        for map_name in (getattr(sim, 'available_maps', []) + ['New Map']):
            r = pygame.Rect(dropdown_rect.x, y, dropdown_rect.width, 30)
            pygame.draw.rect(surface, (255, 255, 255), r)
            pygame.draw.rect(surface, (180, 180, 180), r, 1)
            display_name = map_name.replace('.json', '') if map_name != 'New Map' else map_name
            text = font.render(display_name[:18], True, (0, 0, 0))
            surface.blit(text, (r.x + 5, r.y + 5))
            y += 30

    # Draw edit button
    pygame.draw.rect(surface, (100, 149, 237), edit_button_rect)
    text = font.render('Edit', True, (255, 255, 255))
    text_rect = text.get_rect(center=edit_button_rect.center)
    surface.blit(text, text_rect)
