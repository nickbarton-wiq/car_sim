from enum import Enum, auto
from math import sin, cos, radians, degrees, sqrt, atan2, copysign
from pyray import (
    draw_model_ex, Vector3, RAYWHITE, is_key_down, get_mouse_position, get_screen_to_world_ray, get_ray_collision_quad
)
from raylib import KEY_UP, KEY_DOWN, KEY_SPACE, KEY_RIGHT, KEY_LEFT


class SteeringDirection(Enum):
    NONE = auto()
    LEFT = auto()
    RIGHT = auto()


class AccelerationCommand(Enum):
    NONE = auto()
    ACCELERATE = auto()  # Forward acceleration
    REVERSE = auto()     # Reverse movement
    BRAKE = auto()       # Active braking
    STOP = auto()        # Complete stop


class Navigation:
    def __init__(self):
        self.steering = SteeringDirection.NONE
        self.acceleration = AccelerationCommand.NONE

    def __str__(self):
        return f"Navigation(steering={self.steering.name}, acceleration={self.acceleration.name})"


class ManualControl:
    def __init__(self):
        self.steering = SteeringDirection.NONE
        self.acceleration = AccelerationCommand.NONE

    def __str__(self):
        return f"ManualControl(steering={self.steering.name}, acceleration={self.acceleration.name})"


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

        # Create navigation object
        self.navigation: Navigation | ManualControl = ManualControl()
        self.acceleration = 0.0
        self.steering = 0.0
        self.waypoint: Vector3 | None = None

    def draw(self):
        draw_model_ex(self.model, self.position, Vector3(0, 1, 0), self.angle, Vector3(0.25, 0.25, 0.25), RAYWHITE)

    def handle_steering_wheel_input(self, dt):
        # Handle steering based on navigation or manual input
        if self.navigation.steering == SteeringDirection.RIGHT:
            self.steering -= self.max_steering * dt
        elif self.navigation.steering == SteeringDirection.LEFT:
            self.steering += self.max_steering * dt
        else:
            self.steering = 0
        self.steering = max(-self.max_steering, min(self.steering, self.max_steering))

    def handle_pedal_input(self, dt):
        # Handle acceleration based on navigation or manual input
        if self.navigation.acceleration == AccelerationCommand.ACCELERATE:
            if self.velocity.x < 0:
                self.acceleration = self.brake_deceleration
            else:
                self.acceleration += self.max_acceleration * dt
        elif self.navigation.acceleration == AccelerationCommand.REVERSE:
            if self.velocity.x > 0:
                self.acceleration = -self.brake_deceleration
            else:
                self.acceleration -= self.max_acceleration * dt
        elif self.navigation.acceleration == AccelerationCommand.BRAKE:
            if abs(self.velocity.x) > dt * self.brake_deceleration:
                self.acceleration = -copysign(self.brake_deceleration, self.velocity.x)
            else:
                self.acceleration = -self.velocity.x / dt
        elif self.navigation.acceleration == AccelerationCommand.STOP:
            self.acceleration = 0
            self.velocity.x = 0
        else:
            if abs(self.velocity.x) > dt * self.free_deceleration:
                self.acceleration = -copysign(self.free_deceleration, self.velocity.x)
            else:
                if dt != 0:
                    self.acceleration = -self.velocity.x / dt

        self.acceleration = max(-self.max_acceleration, min(self.acceleration, self.max_acceleration))

    def update(self, dt):
        self.handle_pedal_input(dt)
        # Update velocity based on acceleration
        self.velocity.x += self.acceleration * dt
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        self.handle_steering_wheel_input(dt)
        # Update angle based on steering and velocity
        if self.steering:
            # Reduced turning radius for tighter turns
            turning_radius = self.length / (2 * sin(radians(self.steering)))
            # Multiply by 2 for faster turning rate
            angular_velocity = 2 * self.velocity.x / turning_radius
        else:
            angular_velocity = 0

        self.angle += degrees(angular_velocity) * dt
        self.angle %= 360

        # Update position based on velocity and direction
        self.position.x += self.velocity.x * sin(radians(self.angle)) * dt
        self.position.z += self.velocity.x * cos(radians(self.angle)) * dt

    def manual_navigation(self):
        """Sets the manual control based on the manual key presses"""
        self.navigation = ManualControl()
        if is_key_down(KEY_RIGHT):
            self.navigation.steering = SteeringDirection.RIGHT
        elif is_key_down(KEY_LEFT):
            self.navigation.steering = SteeringDirection.LEFT
        else:
            self.navigation.steering = SteeringDirection.NONE

        if is_key_down(KEY_UP):
            self.navigation.acceleration = AccelerationCommand.ACCELERATE
        elif is_key_down(KEY_DOWN):
            self.navigation.acceleration = AccelerationCommand.REVERSE
        elif is_key_down(KEY_SPACE):
            self.navigation.acceleration = AccelerationCommand.BRAKE
        else:
            self.navigation.acceleration = AccelerationCommand.NONE

    def set_waypoint(self, camera, world):
        """Set the waypoint to the current mouse position"""
        mouse_pos = get_mouse_position()
        ray = get_screen_to_world_ray(mouse_pos, camera.camera)
        collision_point = get_ray_collision_quad(ray, world.p1, world.p2, world.p3, world.p4)
        self.waypoint = Vector3(collision_point.point.x, collision_point.point.y + 0.3, collision_point.point.z)

    def navigate_to_waypoint(self, waypoint):
        """Navigate to the given waypoint"""
        self.navigation = Navigation()
        # Calculate direction vector from car to waypoint
        dx = waypoint.x - self.position.x
        dz = waypoint.z - self.position.z

        # Calculate target angle in degrees (0 is along positive z-axis)
        target_angle = (degrees(atan2(dx, dz)) + 360) % 360

        # Calculate angle difference considering circular nature of angles
        angle_diff = (target_angle - self.angle + 180) % 360 - 180

        # Apply more responsive steering for tighter turns
        # Increased steering factor for faster turning response
        steering_factor = 400
        self.steering = max(-self.max_steering, min(self.max_steering, angle_diff * steering_factor))

        # Calculate distance to waypoint
        distance = sqrt(dx*dx + dz*dz)

        # Set steering direction
        if abs(angle_diff) > 5:
            self.navigation.steering = SteeringDirection.LEFT if angle_diff > 0 else SteeringDirection.RIGHT
        else:
            self.navigation.steering = SteeringDirection.NONE

        # Calculate if we're facing the waypoint or away from it
        # Consider "facing" as within 90 degrees (instead of 100)
        facing_waypoint = abs(angle_diff) < 90

        # Set acceleration command based on distance, orientation, and current velocity
        if distance > 5:
            # Far from waypoint
            self.navigation.acceleration = AccelerationCommand.ACCELERATE
        elif distance > 3 and not facing_waypoint:
            # If we're not moving or moving slowly, it's more efficient to reverse
            if abs(self.velocity.x) < 0.5:
                self.navigation.acceleration = AccelerationCommand.REVERSE
            # If we're already moving backwards significantly, continue reversing
            elif self.velocity.x < -0.5:
                self.navigation.acceleration = AccelerationCommand.REVERSE
            # If we're moving forward but facing away, brake and turn
            else:
                self.navigation.acceleration = AccelerationCommand.BRAKE
        elif distance > 1:
            # Getting closer - reduce speed for more precise turning
            if facing_waypoint:
                # Reduce target speed for better control in tight turns
                target_speed = min(self.max_velocity * 0.5, distance)
                if self.velocity.x < target_speed:
                    self.navigation.acceleration = AccelerationCommand.ACCELERATE
                else:
                    self.navigation.acceleration = AccelerationCommand.NONE
            else:
                # We're facing away from target but close - use reverse
                self.navigation.acceleration = AccelerationCommand.REVERSE
        else:
            # Very close to waypoint
            if abs(self.velocity.x) < 0.1 and distance < 0.5:
                self.navigation.acceleration = AccelerationCommand.STOP
                self.velocity.x = 0
                self.waypoint = None
                self.navigation.steering = SteeringDirection.NONE
                self.navigation.acceleration = AccelerationCommand.NONE
            else:
                self.navigation.acceleration = AccelerationCommand.BRAKE  # Actively brake to stop
