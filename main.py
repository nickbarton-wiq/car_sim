import os
from math import sin, cos, radians, degrees, copysign, sqrt
from pyray import (
    init_window, close_window, window_should_close, get_frame_time, begin_drawing, clear_background, draw_text,
    end_drawing, begin_mode_3d, end_mode_3d, draw_grid, load_model, draw_model_ex, is_key_down,
    Camera3D, Vector3, RAYWHITE, DARKGRAY, is_mouse_button_down,
    get_mouse_position, set_mouse_position, get_screen_width, get_screen_height, disable_cursor, enable_cursor,
    get_mouse_wheel_move_v
)
from raylib import (CAMERA_PERSPECTIVE, KEY_UP, KEY_DOWN, KEY_SPACE, KEY_RIGHT, KEY_LEFT, KEY_W, KEY_A, KEY_S, KEY_D,
                    MOUSE_BUTTON_RIGHT, MOUSE_BUTTON_MIDDLE)


class Car:
    def __init__(self, model, x, z, angle=0, length=4, max_steering=50, max_acceleration=10.0):
        self.model = model
        self.position = Vector3(x, 0, z)
        self.velocity = Vector3()
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
        self.angle %= 360

        # Update position based on velocity and direction
        self.position.x += self.velocity.x * sin(radians(self.angle)) * dt
        self.position.z += self.velocity.x * cos(radians(self.angle)) * dt

    def draw(self):
        draw_model_ex(self.model, self.position, Vector3(0, 1, 0), car.angle, Vector3(0.25, 0.25, 0.25), RAYWHITE)


class FreeCamera:
    def __init__(self):
        self.camera = Camera3D()
        self.camera.position = Vector3(0.0, 10.0, 10.0)
        self.camera.target = Vector3(0.0, 0.0, 0.0)
        self.camera.up = Vector3(0.0, 1.0, 0.0)
        self.camera.fovy = 65.0
        self.camera.projection = CAMERA_PERSPECTIVE

        # Camera control properties
        self.move_speed = 10.0
        self.mouse_sensitivity = 0.2
        self.previous_mouse_position = get_mouse_position()
        self.mouse_right_dragging = False
        self.first_mouse_capture = True  # Flag to track initial mouse capture
        self.zoom_speed = 0.1  # Speed for zooming with mouse wheel

        # Camera rotation angles
        self.yaw = -90.0  # Initial yaw (facing along negative z-axis)
        self.pitch = -45.0  # Initial pitch (looking down a bit)

        # Calculate initial direction vectors
        self._update_camera_vectors()

    def _update_camera_vectors(self):
        # Calculate the new direction vector based on yaw and pitch
        direction = Vector3()
        direction.x = cos(radians(self.yaw)) * cos(radians(self.pitch))
        direction.y = sin(radians(self.pitch))
        direction.z = sin(radians(self.yaw)) * cos(radians(self.pitch))

        # Update camera target
        self.camera.target.x = self.camera.position.x + direction.x
        self.camera.target.y = self.camera.position.y + direction.y
        self.camera.target.z = self.camera.position.z + direction.z

    def handle_keyboard_movement(self, dt):
        # Calculate forward/right vectors for movement
        forward = Vector3(
            self.camera.target.x - self.camera.position.x,
            0,  # We ignore the y component for forward movement to stay on the same plane
            self.camera.target.z - self.camera.position.z
        )

        # Normalize the forward vector
        forward_length = sqrt(forward.x * forward.x + forward.z * forward.z)
        if forward_length > 0:
            forward.x /= forward_length
            forward.z /= forward_length

        # Calculate right vector (cross product of forward and world up)
        right = Vector3(
            forward.z,
            0,
            -forward.x
        )

        # Calculate movement based on input
        move_vec = Vector3(0, 0, 0)

        if is_key_down(KEY_W):
            move_vec.x += forward.x
            move_vec.z += forward.z
        if is_key_down(KEY_S):
            move_vec.x -= forward.x
            move_vec.z -= forward.z
        if is_key_down(KEY_A):
            move_vec.x += right.x
            move_vec.z += right.z
        if is_key_down(KEY_D):
            move_vec.x -= right.x
            move_vec.z -= right.z

        # Vertical movement
        if is_key_down(KEY_SPACE):
            move_vec.y += 1.0

        # Normalize movement vector if not zero
        length = sqrt(move_vec.x * move_vec.x + move_vec.y * move_vec.y + move_vec.z * move_vec.z)
        if length > 0:
            move_vec.x /= length
            move_vec.y /= length
            move_vec.z /= length

        # Apply movement
        self.camera.position.x += move_vec.x * self.move_speed * dt
        self.camera.position.y += move_vec.y * self.move_speed * dt
        self.camera.position.z += move_vec.z * self.move_speed * dt

        # Update target based on new position
        self._update_camera_vectors()

    def handle_mouse_rotation(self):
        current_mouse_position = get_mouse_position()

        # Start dragging when right mouse button is pressed
        if any([is_mouse_button_down(MOUSE_BUTTON_RIGHT), is_mouse_button_down(MOUSE_BUTTON_MIDDLE)]):
            if not self.mouse_right_dragging:
                self.mouse_right_dragging = True
                self.first_mouse_capture = True  # Set flag for first capture
                disable_cursor()  # Hide cursor during rotation
                self.previous_mouse_position = current_mouse_position
            else:
                # Skip the initial mouse capture to prevent jump
                if self.first_mouse_capture:
                    self.first_mouse_capture = False
                    self.previous_mouse_position = current_mouse_position
                    return  # Exit without rotating

                # Calculate mouse movement delta
                dx = current_mouse_position.x - self.previous_mouse_position.x
                dy = current_mouse_position.y - self.previous_mouse_position.y

                # Only update rotation if there is actual mouse movement
                if abs(dx) > 0.0001 or abs(dy) > 0.0001:
                    # Update rotation angles
                    self.yaw += dx * self.mouse_sensitivity
                    self.pitch -= dy * self.mouse_sensitivity  # Inverted for natural control

                    # Constrain pitch to avoid gimbal lock
                    self.pitch = max(-89.0, min(89.0, self.pitch))

                    # Update camera vectors with new rotation
                    self._update_camera_vectors()

                # Reset mouse to the center of the screen to allow for continuous rotation
                center_x = get_screen_width() / 2
                center_y = get_screen_height() / 2
                set_mouse_position(int(center_x), int(center_y))
                self.previous_mouse_position = get_mouse_position()
        else:
            if self.mouse_right_dragging:
                self.mouse_right_dragging = False
                enable_cursor()  # Show cursor after rotation
            self.previous_mouse_position = current_mouse_position

    def handle_mouse_zoom(self, dt):
        # Get mouse wheel movement
        wheel_move = get_mouse_wheel_move_v()
        zoom_amount = wheel_move.y * self.zoom_speed  # Use vertical wheel movement for zooming

        if abs(zoom_amount) > 0:
            # Calculate direction vector (from camera position to target)
            direction = Vector3(
                self.camera.target.x - self.camera.position.x,
                self.camera.target.y - self.camera.position.y,
                self.camera.target.z - self.camera.position.z
            )

            # Normalize direction vector
            dir_length = sqrt(direction.x**2 + direction.y**2 + direction.z**2)
            if dir_length > 0:
                direction.x /= dir_length
                direction.y /= dir_length
                direction.z /= dir_length

                # Move camera position along the direction vector
                # Positive wheel moves camera closer to target (zoom in)
                # Negative wheel moves camera away from target (zoom out)
                self.camera.position.x += direction.x * zoom_amount
                self.camera.position.y += direction.y * zoom_amount
                self.camera.position.z += direction.z * zoom_amount

                # Update camera vectors to ensure proper orientation
                self._update_camera_vectors()

    def update(self, dt):
        self.handle_keyboard_movement(dt)
        self.handle_mouse_rotation()
        self.handle_mouse_zoom(dt)


init_window(1920, 1080, "3D Car prototype")

# model
car_model = load_model(os.path.join("resources", "car_model", "car.glb"))
car = Car(car_model, 0, 0)

camera = FreeCamera()

while not window_should_close():
    dt = get_frame_time()

    # User Input
    # Update camera
    camera.update(dt)

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

    # Draw 3D
    begin_mode_3d(camera.camera)
    draw_grid(100, 1)
    car.draw()
    end_mode_3d()

    # Draw Debug
    draw_text(f"position: x:{car.position.x:.2f}, y:{car.position.y:.2f}, z:{car.position.z:.2f}", 10, 10, 20, DARKGRAY)
    draw_text(f"velocity: x:{car.velocity.x:.2f}, y:{car.velocity.y:.2f}, z:{car.velocity.z:.2f}", 10, 40, 20, DARKGRAY)
    draw_text(f"steering: {car.steering:.2f}", 10, 70, 20, DARKGRAY)
    draw_text(f"angle: {car.angle:.2f}", 10, 100, 20, DARKGRAY)
    end_drawing()
close_window()
