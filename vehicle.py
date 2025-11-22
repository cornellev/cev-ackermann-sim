import math


def calculate_angular_velocity(speed, steering_angle, wheelbase):
    if abs(steering_angle) > 1e-6:
        turning_radius = wheelbase / math.tan(steering_angle)
        return speed / turning_radius
    else:
        return 0


class Vehicle:
    def __init__(self, x=0, y=0):
        # Vehicle parameters in meters
        self.wheelbase = 0.4572*0.8  # L (18 inches)
        self.track_width = 0.3048  # W (12 inches)
        self.length = 0.9144 # 3 feet in meters
        self.width = 0.6096  # 2 feet in meters

        self.max_speed = 2.2352  # m/s (5 mph)
        self.throttle_acceleration = 2.5  # m/s^2
        self.max_acceleration = 2.5  # m/s^2
        self.steering_rate = math.radians(45) # rad/s
        self.max_steering_rate = math.radians(45) # rad/s

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

    def force_update(self, dt, speed, steering_angle):
        """Updates vehicle state with given parameters from trajectory follower. 
        If the given speed exceeds the max velocity, clamp down to max velocity
        If the given steering angle exceeds max steering angle speed, clamp down as well
        """
        if abs(self.speed - speed) > self.max_acceleration * dt:
            speed = self.speed + self.max_acceleration * dt if speed > self.speed else self.speed - self.max_acceleration * dt
        if abs(speed) > self.max_speed:
            speed = self.max_speed if speed > 0 else -self.max_speed
        
        if abs(self.steering_angle - steering_angle) > self.max_steering_rate * dt:
            steering_angle = self.steering_angle + self.max_steering_rate * dt if steering_angle > self.steering_angle else self.steering_angle - self.max_steering_rate * dt
        if abs(steering_angle) > self.max_steering_angle:
            steering_angle = self.max_steering_angle if steering_angle > 0 else -self.max_steering_angle

        self.speed = speed
        self.steering_angle = steering_angle

        angular_velocity = calculate_angular_velocity(self.speed, self.steering_angle, self.wheelbase)
        self.heading += angular_velocity * dt
        # Normalize heading to be within -pi to pi
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi

        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

    def update(self, dt, speed_input, steer_input):
        """Update vehicle state w/ bicycle model"""

        # update speed based on throttle input
        self.speed = max(-self.max_speed, min(self.max_speed, self.speed + speed_input * self.throttle_acceleration * dt))
        if speed_input < 1e-6:
            # natural deceleration
            self.speed *= 0.25 ** dt

        # update effective steering angle based on steering input
        if steer_input != 0:
            self.steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, self.steering_angle + steer_input * self.steering_rate * dt))
        else:
            self.steering_angle = self.steering_angle * 0.9 # natural return to center
        # update position and heading
        angular_velocity = calculate_angular_velocity(self.speed, self.steering_angle, self.wheelbase)

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
