import pygame
from pygame.draw import circle

from constants import SCALE

def circle_line_collision(circle_center, radius, p1, p2):
    """Check if circle collides with line segment p1-p2."""
    cx, cy = circle_center
    x1, y1 = p1
    x2, y2 = p2

    # Vector from p1 to circle center
    dx, dy = x2 - x1, y2 - y1
    fx, fy = cx - x1, cy - y1

    # Project point onto line, clamped between [0,1]
    t = max(0, min(1, (fx * dx + fy * dy) / (dx * dx + dy * dy + 1e-12)))
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    # Distance from circle center to closest point
    dist_sq = (closest_x - cx) ** 2 + (closest_y - cy) ** 2
    return dist_sq <= radius ** 2

class Obstacle:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.color = (255, 165, 0)

    def draw(self, screen, world_to_screen_func):
        """Each obstacle subclass must implement its own drawing method."""
        raise NotImplementedError

    def check_collision(self, vehicle_corners):
        """Each obstacle subclass must implement its own collision logic."""
        raise NotImplementedError

class CircleObstacle(Obstacle):
    def __init__(self, x, y, radius):
        super().__init__(x, y)
        self.radius = radius

    def draw(self, screen, world_to_screen_func):
        screen_pos = world_to_screen_func(self.x, self.y)
        pygame.draw.circle(screen, self.color, screen_pos, int(self.radius * SCALE))

    # TODO fix this
    def check_collision(self, vehicle_corners):
        vehicle_sides = [(vehicle_corners[i], vehicle_corners[(i+1)%4]) for i in range(4)]
        for p1,p2 in vehicle_sides:
            if circle_line_collision((self.x,self.y),self.radius, p1,p2):
                return True
        return False

class CollisionDetector:
    def __init__(self):
        self.tolerance = 0

    def check_collision(self,vehicle_corners,obstacles):
        """Check if vehicle collides with any obstacle"""
        colliding_obstacles = []
        for obs in obstacles:
            if obs.check_collision(vehicle_corners):
                colliding_obstacles.append(obs)
        return colliding_obstacles

