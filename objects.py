import pygame
from pygame.draw import circle

from constants import SCALE, ORANGE, WHITE

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

def on_segment(p, q, r):
    """
    Given three collinear points p, q, r, this function checks if point q
    lies on line segment 'pr'.
    """
    return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
            q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

def orientation(p, q, r):
    """
    Finds the orientation of an ordered triplet (p, q, r).
    The function returns the following values:
    0 --> p, q, and r are collinear
    1 --> Clockwise
    2 --> Counter-clockwise
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - \
          (q[0] - p[0]) * (r[1] - q[1])

    if val == 0:
        return 0  # Collinear
    return 1 if val > 0 else 2  # Clockwise or Counter-clockwise

def lines_intersect(p1, q1, p2, q2):
    """
    The main function that determines if the line segment 'p1q1' and 'p2q2'
    intersect. This algorithm is based on point orientation.
    """
    # Find the four orientations needed for the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case: If the orientations are different, they intersect.
    if o1 != o2 and o3 != o4:
        return True

    # Special Cases for collinear points

    # p1, q1, and p2 are collinear and p2 lies on segment p1q1
    if o1 == 0 and on_segment(p1, p2, q1):
        return True

    # p1, q1, and q2 are collinear and q2 lies on segment p1q1
    if o2 == 0 and on_segment(p1, q2, q1):
        return True

    # p2, q2, and p1 are collinear and p1 lies on segment p2q2
    if o3 == 0 and on_segment(p2, p1, q2):
        return True

    # p2, q2, and q1 are collinear and q1 lies on segment p2q2
    if o4 == 0 and on_segment(p2, q1, q2):
        return True

    # If none of the cases are met, the segments do not intersect
    return False

class Obstacle:
    def __init__(self):
        return

    def draw(self, screen, world_to_screen_func):
        """Each obstacle subclass must implement its own drawing method."""
        raise NotImplementedError

    def check_collision(self, vehicle_corners):
        """Each obstacle subclass must implement its own collision logic."""
        raise NotImplementedError
        
    def to_dict(self):
        """Each obstacle subclass must implement its own serialization."""
        raise NotImplementedError

class CircleObstacle(Obstacle):
    def __init__(self, x, y, radius):
        super().__init__()
        self.color = ORANGE
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen, world_to_screen_func):
        screen_pos = world_to_screen_func(self.x, self.y)
        pygame.draw.circle(screen, self.color, screen_pos, int(self.radius * SCALE))

    def check_collision(self, vehicle_corners):
        vehicle_sides = [(vehicle_corners[i], vehicle_corners[(i+1)%4]) for i in range(4)]
        for p1,p2 in vehicle_sides:
            if circle_line_collision((self.x,self.y),self.radius, p1,p2):
                return True
        return False
        
    def to_dict(self):
        return {'type': 'circle', 'x': self.x, 'y': self.y, 'radius': self.radius}

class PolygonObstacle(Obstacle):
    def __init__(self, vertices, color):
        super().__init__()
        self.vertices = vertices
        self.color = color

    def draw(self, screen, world_to_screen_func):
        screen_points = [world_to_screen_func(x, y) for x, y in self.vertices]
        pygame.draw.polygon(screen, self.color, screen_points)

    def check_collision(self, vehicle_corners):
        vehicle_sides = [(vehicle_corners[i], vehicle_corners[(i+1)%4]) for i in range(4)]
        obstacle_sides = [(self.vertices[i], self.vertices[(i+1)%len(self.vertices)]) for i in range(len(self.vertices))]
        for p1,p2 in vehicle_sides:
            for q1,q2 in obstacle_sides:
                if lines_intersect(p1,p2,q1,q2):
                    return True
        return False
        
    def to_dict(self):
        return {'type': 'polygon', 'vertices': self.vertices, 'color': self.color}

class LineObstacle(PolygonObstacle):
    def __init__(self, start, end, width, color=WHITE):
        # Create a rectangle representing the line with given width
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = (dx**2 + dy**2)**0.5
        if length == 0:
            # Degenerate case: start and end are the same
            vertices = [
                (start[0] - width/2, start[1] - width/2),
                (start[0] + width/2, start[1] - width/2),
                (start[0] + width/2, start[1] + width/2),
                (start[0] - width/2, start[1] + width/2)
            ]
        else:
            ux, uy = dx / length, dy / length
            perp_x, perp_y = -uy * (width / 2), ux * (width / 2)
            vertices = [
                (start[0] + perp_x, start[1] + perp_y),
                (start[0] - perp_x, start[1] - perp_y),
                (end[0] - perp_x, end[1] - perp_y),
                (end[0] + perp_x, end[1] + perp_y)
            ]
        super().__init__(vertices, color)


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

