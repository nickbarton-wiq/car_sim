from abc import abstractmethod, ABC
from enum import Enum, auto
from math import sin, cos, radians, degrees, sqrt, atan2, copysign
from pyray import draw_model_ex, is_key_down, get_mouse_position, get_screen_to_world_ray, get_ray_collision_quad
from pyray import Vector3, RAYWHITE
from raylib import KEY_UP, KEY_DOWN, KEY_SPACE, KEY_RIGHT, KEY_LEFT


class SteeringCommand(Enum):
    NONE = auto()
    LEFT = auto()
    RIGHT = auto()


class AccelerationCommand(Enum):
    NONE = auto()
    ACCELERATE = auto()
    REVERSE = auto()
    BRAKE = auto()
    STOP = auto()


class Driver(ABC):
    def __init__(self):
        self.steering = SteeringCommand.NONE
        self.acceleration = AccelerationCommand.NONE

    @abstractmethod
    def __str__(self):
        pass


class User(Driver):
    def __str__(self):
        return f"User(steering={self.steering.name}, acceleration={self.acceleration.name})"


class AutoPilot(Driver):
    def __str__(self):
        return f"AutoPilot(steering={self.steering.name}, acceleration={self.acceleration.name})"


class Car:
    def __init__(self, model, x, z, angle=0, length=4, max_steering=30, max_acceleration=5.0):
        self.model = model
        self.position = Vector3(x, 0, z)
        self.velocity = Vector3()
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 5
        self.brake_deceleration = 20
        self.free_deceleration = 2

        self.driver: AutoPilot | User = User()
        self.acceleration = 0.0
        self.steering = 0.0
        self.waypoints = []

    def draw(self):
        """Draw the car model"""
        draw_model_ex(self.model, self.position, Vector3(0, 1, 0), self.angle, Vector3(0.25, 0.25, 0.25), RAYWHITE)

    def update(self, dt):
        """Update the car navigation method and position"""
        if self.waypoints:
            self.navigate_to_waypoint(self.waypoints[0])  # Navigate to the first waypoint
        else:
            self.manual_navigation()
        self.update_position(dt)

    def update_position(self, dt):
        """Update the car's position from pedal and steering inputs"""
        self.handle_pedal_input(dt)
        self.handle_steering_wheel_input(dt)

        # Update position based on pedal and steering input
        self.position.x += self.velocity.x * sin(radians(self.angle)) * dt
        self.position.z += self.velocity.x * cos(radians(self.angle)) * dt

    def handle_steering_wheel_input(self, dt):
        """Handle steering inputs"""
        # Process steering input
        if self.driver.steering == SteeringCommand.RIGHT:
            self.steering -= self.max_steering * dt
        elif self.driver.steering == SteeringCommand.LEFT:
            self.steering += self.max_steering * dt
        else:
            self.steering = 0

        # Clamp steering value
        self.steering = max(-self.max_steering, min(self.steering, self.max_steering))

        # Only calculate angular velocity if steering or moving
        if self.steering and self.velocity.x != 0:
            # Calculate turning radius and angular velocity in one step
            angular_velocity = 2 * self.velocity.x * sin(radians(self.steering)) / self.length
            self.angle += degrees(angular_velocity) * dt
            self.angle %= 360

    def handle_pedal_input(self, dt):
        """Handle acceleration inputs"""
        if self.driver.acceleration == AccelerationCommand.ACCELERATE:
            if self.velocity.x < 0:
                self.acceleration = self.brake_deceleration
            else:
                self.acceleration += self.max_acceleration * dt
        elif self.driver.acceleration == AccelerationCommand.REVERSE:
            if self.velocity.x > 0:
                self.acceleration = -self.brake_deceleration
            else:
                self.acceleration -= self.max_acceleration * dt
        elif self.driver.acceleration == AccelerationCommand.BRAKE:
            if abs(self.velocity.x) > dt * self.brake_deceleration:
                self.acceleration = -copysign(self.brake_deceleration, self.velocity.x)
            else:
                self.acceleration = -self.velocity.x / dt
        elif self.driver.acceleration == AccelerationCommand.STOP:
            self.acceleration = 0
            self.velocity.x = 0
        else:
            if abs(self.velocity.x) > dt * self.free_deceleration:
                self.acceleration = -copysign(self.free_deceleration, self.velocity.x)
            else:
                if dt != 0:
                    self.acceleration = -self.velocity.x / dt
        self.acceleration = max(-self.max_acceleration, min(self.acceleration, self.max_acceleration))

        # Update velocity based on acceleration
        self.velocity.x += self.acceleration * dt
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

    def manual_navigation(self):
        """Sets the pedal and steering inputs based on the manual key presses"""
        self.driver = User()
        if is_key_down(KEY_RIGHT):
            self.driver.steering = SteeringCommand.RIGHT
        elif is_key_down(KEY_LEFT):
            self.driver.steering = SteeringCommand.LEFT
        else:
            self.driver.steering = SteeringCommand.NONE

        if is_key_down(KEY_UP):
            self.driver.acceleration = AccelerationCommand.ACCELERATE
        elif is_key_down(KEY_DOWN):
            self.driver.acceleration = AccelerationCommand.REVERSE
        elif is_key_down(KEY_SPACE):
            self.driver.acceleration = AccelerationCommand.BRAKE
        else:
            self.driver.acceleration = AccelerationCommand.NONE

    def add_waypoint(self, camera, world):
        """Add a waypoint to the list of waypoints"""
        mouse_pos = get_mouse_position()
        ray = get_screen_to_world_ray(mouse_pos, camera.camera)
        collision_point = get_ray_collision_quad(ray, world.p1, world.p2, world.p3, world.p4)
        new_waypoint = Vector3(collision_point.point.x, collision_point.point.y + 0.3, collision_point.point.z)
        self.waypoints.append(new_waypoint)

    def navigate_to_waypoint(self, waypoint):
        """Sets the pedal and steering inputs to navigate to the waypoint"""
        self.driver = AutoPilot()
        # Calculate direction vector from car to waypoint
        dx = waypoint.x - self.position.x
        dz = waypoint.z - self.position.z

        # Calculate target angle in degrees (0 is along positive z-axis)
        target_angle = (degrees(atan2(dx, dz)) + 360) % 360

        # Calculate angle difference considering circular nature of angles
        angle_diff = (target_angle - self.angle + 180) % 360 - 180

        # Set steering direction
        if angle_diff > 5:
            self.driver.steering = SteeringCommand.LEFT
        elif angle_diff < 5:
            self.driver.steering = SteeringCommand.RIGHT
        else:
            self.driver.steering = SteeringCommand.NONE

        # Calculate distance to waypoint
        distance = sqrt(dx*dx + dz*dz)

        # Calculate if we're facing the waypoint or away from it. Consider "facing" as within 90 degrees
        facing_waypoint = abs(angle_diff) < 90

        # Check if this is the last waypoint
        is_last_waypoint = len(self.waypoints) == 1
        if is_last_waypoint:
            if distance >= 3:
                # Far from waypoint - accelerate
                self.driver.acceleration = AccelerationCommand.ACCELERATE
            if distance < 3:
                # Getting closer - reduce speed for more precise turning
                if facing_waypoint:
                    # Reduce target speed for better control in tight turns
                    speed_factor = 0.5
                    target_speed = min(self.max_velocity * speed_factor, distance)
                    if self.velocity.x < target_speed:
                        self.driver.acceleration = AccelerationCommand.ACCELERATE
                    else:
                        self.driver.acceleration = AccelerationCommand.NONE
                else:
                    # We're facing away from target but close - use reverse
                    self.driver.acceleration = AccelerationCommand.REVERSE
            if distance <= 1.5:
                # Very close to waypoint
                if abs(self.velocity.x) < 0.1:
                    self.waypoints.pop(0)
                    self.driver.acceleration = AccelerationCommand.STOP
                    self.velocity.x = 0
                    self.driver.steering = SteeringCommand.NONE
                    self.driver.acceleration = AccelerationCommand.NONE
                else:
                    self.driver.acceleration = AccelerationCommand.BRAKE
        else:
            if not facing_waypoint and distance < 3:
                self.driver.acceleration = AccelerationCommand.REVERSE
            else:
                self.driver.acceleration = AccelerationCommand.ACCELERATE

            # if we are close to the waypoint, remove it and move to the next one
            if distance < 1:
                self.waypoints.pop(0)
