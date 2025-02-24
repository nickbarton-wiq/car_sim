import os
from math import sin, cos, radians, degrees, copysign
from pyray import (
    init_window, close_window, window_should_close, get_frame_time, begin_drawing, clear_background, draw_text,
    end_drawing, begin_mode_3d, end_mode_3d, draw_grid, load_model, draw_model_ex, is_key_down,
    Camera3D, Vector3, RAYWHITE, DARKGRAY,
)
from raylib import CAMERA_PERSPECTIVE, KEY_UP, KEY_DOWN, KEY_SPACE, KEY_RIGHT, KEY_LEFT


class Car:
    def __init__(self, model, x, z, angle=0, length=4, max_steering=50, max_acceleration=10.0):
        self.model = model
        self.position = Vector3(x, 0, z)
        self.velocity = Vector3(0, 0, 0)
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 10
        self.brake_deceleration = 10
        self.free_deceleration = 2

        self.acceleration = 0.0
        self.steering = 0.0

    def update(self, dt):
        # Update velocity based on acceleration
        self.velocity.x += self.acceleration * dt
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        # Update angle based on steering and velocity
        if self.steering:
            turning_radius = self.length / sin(radians(self.steering))
            angular_velocity = self.velocity.x / turning_radius
        else:
            angular_velocity = 0

        self.angle += degrees(angular_velocity) * dt

        # Update position based on velocity and direction
        self.position.x += self.velocity.x * sin(radians(self.angle)) * dt
        self.position.z += self.velocity.x * cos(radians(self.angle)) * dt


init_window(1920, 1080, "3D Car prototype")

# camera
camera = Camera3D()
camera.position = Vector3(0.0, 5.0, 10.0)
camera.target = Vector3(0.0, 0.0, 0.0)
camera.up = Vector3(0.0, 0.0, -1.0)
camera.fovy = 60.0
camera.projection = CAMERA_PERSPECTIVE

# model
car_model = load_model(os.path.join("resources", "car_model", "car.glb"))
car = Car(car_model, 0, 0)

while not window_should_close():
    dt = get_frame_time()

    # User Input
    if is_key_down(KEY_UP):
        if car.velocity.x < 0:
            car.acceleration = car.brake_deceleration
        else:
            car.acceleration += 10 * dt
    elif is_key_down(KEY_DOWN):
        if car.velocity.x > 0:
            car.acceleration = -car.brake_deceleration
        else:
            car.acceleration -= 1 * dt
    elif is_key_down(KEY_SPACE):
        if abs(car.velocity.x) > dt * car.brake_deceleration:
            car.acceleration = -copysign(car.brake_deceleration, car.velocity.x)
        else:
            car.acceleration = -car.velocity.x / dt
    else:
        if abs(car.velocity.x) > dt * car.free_deceleration:
            car.acceleration = -copysign(car.free_deceleration, car.velocity.x)
        else:
            if dt != 0:
                car.acceleration = -car.velocity.x / dt
    car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))

    if is_key_down(KEY_RIGHT):
        car.steering -= 30 * dt
    elif is_key_down(KEY_LEFT):
        car.steering += 30 * dt
    else:
        car.steering = 0
    car.steering = max(-car.max_steering, min(car.steering, car.max_steering))

    # Logic
    car.update(dt)

    # Drawing
    clear_background(RAYWHITE)
    begin_drawing()

    begin_mode_3d(camera)
    draw_grid(100, 1)
    draw_model_ex(car.model, car.position, Vector3(0, 1, 0), car.angle, Vector3(0.5, 0.5, 0.5), RAYWHITE)
    end_mode_3d()
    draw_text(f"position: x:{car.position.x}, y:{car.position.y}, z:{car.position.z}", 10, 10, 20, DARKGRAY)
    draw_text(f"velocity: x:{car.velocity.x}, y:{car.velocity.y}, z:{car.velocity.z}", 10, 40, 20, DARKGRAY)
    draw_text(f"steering: {car.steering:.2f}", 10, 70, 20, DARKGRAY)
    draw_text(f"angle: {car.angle}", 10, 100, 20, DARKGRAY)
    end_drawing()
close_window()
