#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import threading
import random
import json

# ===================== Eye Class =====================
class Eye:
    def __init__(self, canvas, eye_index):
        self.canvas = canvas
        self.eye_index = eye_index
        self.eye = None
        self.pupil = None
        self.eyelid = None
        self.eye_radius = 0
        self.pupil_radius = 0
        self.pupil_move_radius = 0
        self.center_x = 0
        self.center_y = 0
        self.target_dx = 0
        self.target_dy = 0
        self.current_dx = 0
        self.current_dy = 0
        self.eye_size = 0
        self.blinking = False
        self.eyelid_position = 0  # 0 = open, 1 = closed

    def draw(self, center_x, center_y, eye_size):
        self.eye_radius = eye_size // 2
        self.pupil_radius = eye_size // 6
        self.pupil_move_radius = self.eye_radius - self.pupil_radius
        self.center_x = center_x
        self.center_y = center_y
        self.eye_size = eye_size

        self._draw_eye()
        self._draw_eyelid()

    def _draw_eye(self):
        if self.eye:
            self.canvas.delete(self.eye)
        if self.pupil:
            self.canvas.delete(self.pupil)

        self.eye = self.canvas.create_oval(
            self.center_x - self.eye_radius, self.center_y - self.eye_radius,
            self.center_x + self.eye_radius, self.center_y + self.eye_radius,
            fill='white', outline='black', width=3)

        self.pupil = self.canvas.create_oval(
            self.center_x - self.pupil_radius + self.current_dx,
            self.center_y - self.pupil_radius + self.current_dy,
            self.center_x + self.pupil_radius + self.current_dx,
            self.center_y + self.pupil_radius + self.current_dy,
            fill='black'
        )

    def _draw_eyelid(self):
        if self.eyelid:
            self.canvas.delete(self.eyelid)

        eye_top = self.center_y - self.eye_radius
        eye_bottom = self.center_y + self.eye_radius
        lid_height = (eye_bottom - eye_top) * self.eyelid_position

        self.eyelid = self.canvas.create_rectangle(
            self.center_x - self.eye_radius, eye_top,
            self.center_x + self.eye_radius, eye_top + lid_height,
            fill='black', outline=''
        )

    def update_pupil_position(self):
        if self.pupil is None:
            return
        step = 0.15
        self.current_dx += (self.target_dx - self.current_dx) * step
        self.current_dy += (self.target_dy - self.current_dy) * step
        self._draw_eye()
        self._draw_eyelid()

    def look_at_3d_point(self, x, y, z):
        if z == 0:
            z = 0.01
        proj_x = x / z
        proj_y = y / z
        dx = max(-1, min(1, proj_x)) * self.pupil_move_radius
        dy = max(-1, min(1, proj_y)) * self.pupil_move_radius
        self.target_dx = dx
        self.target_dy = dy

    def start_blink(self):
        if self.blinking:
            return
        self.blinking = True
        self.animate_eyelid(direction=1)  # close

    def animate_eyelid(self, direction):
        def step():
            step_size = 0.2
            if direction == 1:
                self.eyelid_position = min(1.0, self.eyelid_position + step_size)
            else:
                self.eyelid_position = max(0.0, self.eyelid_position - step_size)

            self._draw_eyelid()

            if (direction == 1 and self.eyelid_position < 1.0) or (direction == -1 and self.eyelid_position > 0.0):
                root.after(30, step)
            else:
                if direction == 1:
                    root.after(100, lambda: self.animate_eyelid(direction=-1))
                else:
                    self.blinking = False
        step()

# ===================== ROS 2 Node =====================
class EyesNode(Node):
    def __init__(self):
        super().__init__('eyes')
        self.subscription = self.create_subscription(
            String,
            'eth_gaze',
            self.gaze_callback,
            10
        )
        self.latest_coords = (0.0, 0.0, 2.0)  # default position

    def gaze_callback(self, msg):
        try:
            data = json.loads(msg.data)
            cx = data.get('center_x', 0.0)
            cy = data.get('center_y', 0.0)
            cz = data.get('center_z', 2.0)
            self.latest_coords = (float(cx), float(cy), float(cz))
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse JSON from eth_gaze")

    def get_latest_coords(self):
        return self.latest_coords

# ===================== GUI Setup =====================
root = tk.Tk()
root.title("3D Gaze Robot Eyes ROS2 NODE")

notebook = ttk.Notebook(root)
notebook.pack(fill=tk.BOTH, expand=True)

eye_frame = tk.Frame(notebook)
canvas = tk.Canvas(eye_frame, width=1280, height=400, bg='black')
canvas.pack(fill=tk.BOTH, expand=True)
notebook.add(eye_frame, text="Robot Eyes")

eyes = [Eye(canvas, 0), Eye(canvas, 1)]

def on_resize(event):
    canvas_width = event.width
    canvas_height = event.height
    canvas.delete("all")
    eye_size = min(canvas_height * 0.8, canvas_width / 3 * 0.8)
    eye_gap = canvas_width * 0.1
    total_eye_width = eye_size * 2 + eye_gap
    start_x = (canvas_width - total_eye_width) / 2

    left_center_x = start_x + eye_size / 2
    right_center_x = left_center_x + eye_size + eye_gap
    center_y = canvas_height / 2

    eyes[0].draw(left_center_x, center_y, int(eye_size))
    eyes[1].draw(right_center_x, center_y, int(eye_size))

canvas.bind('<Configure>', on_resize)

def update_animation():
    x, y, z = ros_node.get_latest_coords()
    for eye in eyes:
        eye.look_at_3d_point(x, y, z)
        eye.update_pupil_position()
    root.after(16, update_animation)

def random_blink_loop():
    def blink():
        for eye in eyes:
            eye.start_blink()
        delay = random.randint(2000, 5000)
        root.after(delay, blink)
    blink()

# ===================== Main Entry =====================
def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = EyesNode()

    # Run ROS 2 spinning in a background thread
    threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

    # Start GUI animations
    update_animation()
    random_blink_loop()

    # Tkinter mainloop in main thread
    root.mainloop()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
