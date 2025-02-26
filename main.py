import os
from math import sqrt
from pyray import (
    init_window, close_window, window_should_close, get_frame_time, begin_drawing, clear_background, draw_text,
    end_drawing, begin_mode_3d, end_mode_3d, load_model, draw_model_ex, load_model_from_mesh, gen_mesh_cone, GREEN,
    Vector3, RAYWHITE, DARKGRAY, is_mouse_button_pressed, RED
)
from raylib import MOUSE_BUTTON_LEFT

from src.camera import FreeCamera
from src.car import Car
from src.world import World


init_window(1920, 1080, "3D Car prototype")

# World
world = World(Vector3(-50, 0, -50), Vector3(50, 0, -50), Vector3(50, 0, 50), Vector3(-50, 0, 50))

# Car
car_model = load_model(os.path.join("resources", "car_model", "car.glb"))
car = Car(car_model, 0, 0, max_steering=40)

# Waypoint
waypoint_mesh = gen_mesh_cone(0.5, 1.0, 20)
waypoint_model = load_model_from_mesh(waypoint_mesh)

# Camera
camera = FreeCamera()

while not window_should_close():
    dt = get_frame_time()

    # Update camera
    camera.update(dt)

    # Check for waypoint setting
    if is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
        car.add_waypoint(camera, world)

    # Update car position
    car.update(dt)

    # Drawing
    clear_background(RAYWHITE)
    begin_drawing()

    # Draw 3D
    begin_mode_3d(camera.camera)
    world.draw_floor()  # Draw the floor
    world.draw_grid()
    car.draw()

    # Draw waypoints - first one green, rest are red
    if car.waypoints:
        # Draw first waypoint (current target) in green
        draw_model_ex(waypoint_model, car.waypoints[0], Vector3(1, 0, 0), 180, Vector3(0.3, 0.3, 0.3), GREEN)

        # Draw other waypoints in red
        for waypoint in car.waypoints[1:]:
            draw_model_ex(waypoint_model, waypoint, Vector3(1, 0, 0), 180, Vector3(0.3, 0.3, 0.3), RED)

    end_mode_3d()

    # Draw debug data
    draw_text(f"position: x:{car.position.x:.2f}, y:{car.position.y:.2f}, z:{car.position.z:.2f}", 10, 10, 20, DARKGRAY)
    draw_text(f"velocity: x:{car.velocity.x:.2f}, y:{car.velocity.y:.2f}, z:{car.velocity.z:.2f}", 10, 40, 20, DARKGRAY)
    draw_text(f"steering: {car.steering:.2f}", 10, 70, 20, DARKGRAY)
    draw_text(f"angle: {car.angle:.2f}", 10, 100, 20, DARKGRAY)

    if car.waypoints:
        draw_text(f"navigation: {car.navigation}", 10, 130, 20, DARKGRAY)
        current_waypoint = car.waypoints[0]
        distance = sqrt((current_waypoint.x - car.position.x)**2 + (current_waypoint.z - car.position.z)**2)
        draw_text(f"distance to next waypoint: {distance:.2f}", 10, 160, 20, DARKGRAY)
        draw_text(f"waypoints remaining: {len(car.waypoints)}", 10, 190, 20, DARKGRAY)

    end_drawing()
close_window()
