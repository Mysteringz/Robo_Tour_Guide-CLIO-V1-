import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import threading
import numpy as np
import random
import time

class Eye:
    def __init__(self, canvas, eye_index):
        self.canvas = canvas
        self.eye_index = eye_index
        self.eye_shape = None
        self.pupil = None

        self.eye_radius = 0
        self.pupil_radius = 0
        self.pupil_move_radius = 0
        self.eye_size = 0

        # Eye position
        self.base_center_x = 0
        self.base_center_y = 0
        self.center_x = 0
        self.center_y = 0

        # Target offsets (whole eye movement)
        self.target_ex = 0
        self.target_ey = 0
        self.current_ex = 0
        self.current_ey = 0

        # Pupil offsets
        self.target_dx = 0
        self.target_dy = 0
        self.current_dx = 0
        self.current_dy = 0

        # Eyelid
        self.eyelid_position = 0.0
        self.top_lid = None
        self.bottom_lid = None
        self.blinking = False

    def draw(self, center_x, center_y, eye_size):
        """Initialize base eye position"""
        self.base_center_x = center_x
        self.base_center_y = center_y
        self.eye_radius = eye_size // 2
        self.pupil_radius = eye_size // 10
        self.pupil_move_radius = self.eye_radius - self.pupil_radius
        self.eye_size = eye_size
        self._redraw_eye()

    def _redraw_eye(self):
        """Draw eye + pupil at current position"""
        if self.eye_shape:
            self.canvas.delete(self.eye_shape)
        if self.pupil:
            self.canvas.delete(self.pupil)

        self.center_x = self.base_center_x + self.current_ex
        self.center_y = self.base_center_y + self.current_ey

        corner_radius = self.eye_radius * 0.5
        pupil_size = self.eye_radius * 1.8

        # Outer eye
        self.eye_shape = self._create_rounded_rect(
            self.center_x - self.eye_radius,
            self.center_y - self.eye_radius,
            self.center_x + self.eye_radius,
            self.center_y + self.eye_radius,
            corner_radius * 0.7, fill="#00B9B9", outline=''
        )

        # Pupil
        self.pupil = self._create_rounded_rect(
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

    def update_position(self):
        """Smoothly update both eye offset and pupil"""
        step = 0.15

        # Smooth outer-eye movement
        self.current_ex += (self.target_ex - self.current_ex) * step
        self.current_ey += (self.target_ey - self.current_ey) * step

        # Smooth pupil movement
        self.current_dx += (self.target_dx - self.current_dx) * step
        self.current_dy += (self.target_dy - self.current_dy) * step

        # Redraw
        self._redraw_eye()

        # Move pupil relative to new center
        corner_radius = self.eye_radius * 0.5
        pupil_size = self.eye_radius * 1.8
        points = [
            self.center_x - pupil_size / 2 + self.current_dx + corner_radius * 0.5, self.center_y - pupil_size / 2 + self.current_dy,
            self.center_x + pupil_size / 2 + self.current_dx - corner_radius * 0.5, self.center_y - pupil_size / 2 + self.current_dy,
            self.center_x + pupil_size / 2 + self.current_dx, self.center_y - pupil_size / 2 + self.current_dy + corner_radius * 0.5,
            self.center_x + pupil_size / 2 + self.current_dx, self.center_y + pupil_size / 2 + self.current_dy - corner_radius * 0.5,
            self.center_x + pupil_size / 2 + self.current_dx - corner_radius * 0.5, self.center_y + pupil_size / 2 + self.current_dy,
            self.center_x - pupil_size / 2 + self.current_dx + corner_radius * 0.5, self.center_y + pupil_size / 2 + self.current_dy,
            self.center_x - pupil_size / 2 + self.current_dx, self.center_y + pupil_size / 2 + self.current_dy - corner_radius * 0.5,
            self.center_x - pupil_size / 2 + self.current_dx, self.center_y - pupil_size / 2 + self.current_dy + corner_radius * 0.5
        ]
        self.canvas.coords(self.pupil, *points)

    def look_at_3d_point(self, x, y, z):
        """Set pupil target"""
        if z == 0:
            z = 0.01
        proj_x = x / z
        proj_y = y / z
        dx = max(-1, min(1, proj_x)) * self.pupil_move_radius
        dy = max(-1, min(1, proj_y)) * self.pupil_move_radius
        self.target_dx = dx
        self.target_dy = dy

    def set_eye_offset(self, dx, dy):
        """Set whole-eye offset target"""
        self.target_ex = dx * 80   # scaling factor for face tracking
        self.target_ey = dy * 50

    # ---- Eyelids ----
    def _draw_eyelids(self):
        if self.top_lid:
            self.canvas.delete(self.top_lid)
        if self.bottom_lid:
            self.canvas.delete(self.bottom_lid)
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        lid_height = int(h * self.eyelid_position / 2)
        self.top_lid = self.canvas.create_rectangle(0, 0, w, lid_height, fill='black', outline='')
        self.bottom_lid = self.canvas.create_rectangle(0, h - lid_height, w, h, fill='black', outline='')

    def _animate_blink(self, direction):
        step = 0.2
        self.eyelid_position += direction * step
        self.eyelid_position = max(0.0, min(1.0, self.eyelid_position))
        self._draw_eyelids()
        if direction == 1 and self.eyelid_position < 1.0:
            self.canvas.after(30, lambda: self._animate_blink(1))
        elif direction == -1 and self.eyelid_position > 0.0:
            self.canvas.after(30, lambda: self._animate_blink(-1))
        elif direction == 1:  # Fully closed
            self.canvas.after(100, lambda: self._animate_blink(-1))
        else:
            self.blinking = False

    def start_fullscreen_blink(self):
        if self.blinking:
            return
        self.blinking = True
        self._animate_blink(1)

# ---- GUI ----
root = tk.Tk()
root.title("3D Gaze Robot Eyes with Face Tracking")

notebook = ttk.Notebook(root)
notebook.pack(fill=tk.BOTH, expand=True)

# Eye Animation Tab
eye_frame = tk.Frame(notebook)
canvas = tk.Canvas(eye_frame, width=1280, height=400, bg='black')
canvas.pack(fill=tk.BOTH, expand=True)
notebook.add(eye_frame, text="Robot Eyes")

# Face Tracking Tab
video_frame = tk.Frame(notebook)
video_label = tk.Label(video_frame)
video_label.pack()
notebook.add(video_frame, text="Face Tracking")

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

def look_at_person(x, y, z):
    for eye in eyes:
        eye.look_at_3d_point(x, y, z)

def update_animation():
    for eye in eyes:
        eye.update_position()
    root.after(16, update_animation)

def random_blink_loop():
    def blink():
        for eye in eyes:
            eye.start_fullscreen_blink()
        delay = random.randint(2000, 5000)
        root.after(delay, blink)
    blink()

def face_tracking_loop():
    cap = cv2.VideoCapture(0)
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    frame_center_x = 640 / 2
    frame_center_y = 480 / 2
    dx, dy, dz = 0, 0, 2

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        if len(faces) > 0:
            x, y, w, h = faces[0]
            face_center_x = x + w / 2
            face_center_y = y + h / 2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (int(face_center_x), int(face_center_y)), 5, (0, 0, 255), -1)

            dx = -(face_center_x - frame_center_x) / frame_center_x + 2
            dy = (face_center_y - frame_center_y) / frame_center_y - 1
            dz = 5

            look_at_person(dx, dy, dz)
            for eye in eyes:
                eye.set_eye_offset(dx, dy)
        else:
            dx, dy, dz = 0, 0, 2
            look_at_person(dx, dy, dz)
            for eye in eyes:
                eye.set_eye_offset(0, 0)

        # Draw XYZ
        cv2.putText(frame, "Estimated 3D Head Pos:", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"x: {dx:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"y: {dy:.2f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"z: {dz:.2f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Convert frame to ImageTk
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb_frame)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)

        time.sleep(0.03)

# ---- Start ----
update_animation()
random_blink_loop()
threading.Thread(target=face_tracking_loop, daemon=True).start()
root.mainloop()
