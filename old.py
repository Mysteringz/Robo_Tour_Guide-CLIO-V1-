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
import re

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
        self.pupil_radius = eye_size // 5
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

        corner_radius = self.eye_radius * 0.5
        pupil_size = self.eye_radius * 1.8

        self.eye = self.canvas._create_rounded_rect(
            self.center_x - self.eye_radius,
            self.center_y - self.eye_radius,
            self.center_x + self.eye_radius,
            self.center_y + self.eye_radius,
            corner_radius * 0.7, fill="#00B9B9", outline=''
        )

        self.pupil = self.canvas._create_rounded_rect(
            self.center_x - pupil_size / 2,
            self.center_y - pupil_size / 2,
            self.center_x + pupil_size / 2,
            self.center_y + pupil_size / 2,
            corner_radius * 1, fill='#00CCCC', outline=''
        )

    def _create_rounded_rect(self, x1, y1, x2, y2, r=60, **kwargs):
        points = [
            x1 + r, y1, x2 - r, y1, x2, y1 + r,
            x2, y2 - r, x2 - r, y2, x1 + r, y2,
            x1, y2 - r, x1, y1 + r
        ]
        return self.canvas.create_polygon(points, smooth=True, **kwargs)

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
            '/pzgaze/face_info',
            self.gaze_callback,
            10
        )
        self.latest_coords = (0.0, 0.0, 2.0)  # default position
        self.target_face_id = 0

    def gaze_callback(self, msg):
        try:
            face_data = self.parse_face_info(msg.data)
            if not face_data:
                return
            
            target_face = None
            for face in face_data:
                if face['face_id'] == self.target_face_id:
                    target_face = face
                    break

            cx = -target_face['center_x']
            cy = target_face['center_y']
            cz = target_face['center_z']
            
            self.latest_coords = (float(cx), float(cy), float(cz))
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse JSON from eth_gaze")
    
    def parse_face_info(self, data: str) -> list:
        faces = []
        
        face_count_match = re.search(r'faces_count:(\d+)', data)
        if not face_count_match:
            return faces
        
        face_count = int(face_count_match.group(1))
        
        for i in range(face_count):
            face_pattern = rf'face_{i}:([^|]+)'
            face_match = re.search(face_pattern, data)
            
            if face_match:
                face_info_str = face_match.group(1)
                face_info = self._parse_single_face(face_info_str, i)
                if face_info:
                    faces.append(face_info)
        
        return faces
    
    def _parse_single_face(self, face_str: str, face_id: int) -> dict:
        try:
            pitch_match = re.search(r'pitch=([-\d.]+)', face_str)
            yaw_match = re.search(r'yaw=([-\d.]+)', face_str)
            distance_match = re.search(r'distance=([-\d.]+)', face_str)
            center_match = re.search(r'center=\(([-\d.]+),([-\d.]+),([-\d.]+)\)', face_str)
            gaze_pitch_match = re.search(r'gaze_pitch=([-\d.]+)', face_str)
            gaze_yaw_match = re.search(r'gaze_yaw=([-\d.]+)', face_str)
            
            if not all([pitch_match, yaw_match, distance_match, center_match]):
                return None
            
            return {
                'face_id': face_id,
                'head_pitch': float(pitch_match.group(1)),
                'head_yaw': float(yaw_match.group(1)),
                'distance': float(distance_match.group(1)),
                'center_x': float(center_match.group(1)),
                'center_y': float(center_match.group(2)),
                'center_z': float(center_match.group(3)),
                'gaze_pitch': float(gaze_pitch_match.group(1)) if gaze_pitch_match else None,
                'gaze_yaw': float(gaze_yaw_match.group(1)) if gaze_yaw_match else None
            }
        except Exception as e:
            self.get_logger().error(f"Error parsing face info: {str(e)}")
            return None

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
        delay = random.randint(10000, 15000)
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