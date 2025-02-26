from pyray import Color, Vector3, Vector2, draw_plane, draw_grid


class World:
    def __init__(self, p1, p2, p3, p4):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        self.floor_color = Color(150, 150, 150, 255)  # Grey color for the floor

    def draw_floor(self):
        # Draw the floor as a quad using the four corners
        draw_plane(Vector3(0, 0, 0), Vector2(100, 100), self.floor_color)  # Simple plane centered at origin

    def draw_grid(self):
        draw_grid(100, 1)  # Draw a grid on the floor
