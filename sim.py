import pygame
import math
import rclpy
from rclpy.node import Node
from objects import CircleObstacle, CollisionDetector, PolygonObstacle, LineObstacle
from constants import SCALE, SCREEN_WIDTH, SCREEN_HEIGHT, LIGHT_GRAY, LIGHT_BLUE, BLACK, RED, YELLOW, WHITE, FPS, GRAY
from sim_publisher import VehiclePublisher

pygame.init()

class Vehicle:
    def __init__(self, x=0, y=0):
        # Vehicle parameters in meters
        self.wheelbase = 0.9144  # L
        self.track_width = 0.6096  # w
        self.length = 0.9144 # 3 feet in meters
        self.width = 0.6096 # 2 feet in meters

        self.max_speed = 5.0  # m/s
        self.throttle_acceleration = 2.0  # m/s^2
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
        # TODO: derive max steering angle
        return math.radians(30) # conservative placeholder

class Simulator:
    def __init__(self):
        rclpy.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Vehicle Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 18)
        self.vehicle = Vehicle(0, 0)

        self.obstacles = [
            # CircleObstacle(10, 5, 0.4572), # 18 inches in meters (this is a cone)
            # CircleObstacle(-5, 10, 0.4572),
            # CircleObstacle(15, -8, 0.4572),
            # # another obstacle next to that one to test collision
            # CircleObstacle(15, -10, 0.4572),
            # # big cube for testing
            # # PolygonObstacle([(-3, -3), (-1, -3), (-1, -1), (-3, -1)], (255, 165, 0)),
            # # 2 road lines of thickess 0.15m from (-10,0) to (10,0) 10 feet wide
            # LineObstacle((-10, 0.75), (10, 0.75), 0.15),
            # LineObstacle((-10, -0.75), (10, -0.75), 0.15)
            #
            # make two cones one above another
            CircleObstacle(2, 5, 0.4572), # 18 inches in meters (this is a cone)
            CircleObstacle(2, 6.2, 0.4572),
            CircleObstacle(5.5, 5.2, 0.4572),
            CircleObstacle(5.5, 6, 0.4572),
            CircleObstacle(5.5, 7, 0.4572),

            CircleObstacle(8, 4.5, 0.4572),
            CircleObstacle(8, 3.5, 0.4572),

            # make a road line directly above the top cones
            LineObstacle((-10, 7.75), (20, 7.75), 0.15),



        ]
        self.collision_detector = CollisionDetector()
        self.is_colliding = False

        self.camera_x = 0
        self.camera_y = 0

        self.pose_publisher = VehiclePublisher()

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
        """Converts world coordinates (meters) to screen coordinates (pixels)."""
        screen_x = int((x - self.camera_x) * SCALE + SCREEN_WIDTH / 2)
        # Invert y-axis for Pygame's coordinate system
        screen_y = int(-(y - self.camera_y) * SCALE + SCREEN_HEIGHT / 2)
        return screen_x, screen_y

    def draw_grid(self):
        """Draws a grid on the screen for reference."""
        grid_size = 50 # meters
        for i in range(-grid_size, grid_size + 1):
            # Vertical lines
            start_pos = self.world_to_screen(i, -grid_size)
            end_pos = self.world_to_screen(i, grid_size)
            pygame.draw.line(self.screen, LIGHT_GRAY, start_pos, end_pos, 1)
            # Horizontal lines
            start_pos = self.world_to_screen(-grid_size, i)
            end_pos = self.world_to_screen(grid_size, i)
            pygame.draw.line(self.screen, LIGHT_GRAY, start_pos, end_pos, 1)

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
        for obs in self.obstacles:
            obs.draw(self.screen,self.world_to_screen)

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


    def run(self):
        """Main simulation loop"""
        running = True
        dt = 1.0 / FPS

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            speed_input, steer_input = self.handle_input()
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
                except Exception as e:
                    print(f"Failed to publish pose: {e}")
                self._last_publish_time = now

            # Rendering
            self.screen.fill(GRAY)
            self.draw_grid()
            self.draw_vehicle()
            self.draw_hud()
            self.draw_obstacles()

            pygame.display.flip()
            self.clock.tick(FPS)
        pygame.quit()
        try:
            self.pose_publisher.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    simulator = Simulator()
    simulator.run()