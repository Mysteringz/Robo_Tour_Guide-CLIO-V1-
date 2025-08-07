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

# ---- GUI and Layout ----
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
        eye.update_pupil_position()
    root.after(16, update_animation)

def random_blink_loop():
    def blink():
        for eye in eyes:
            eye.start_blink()
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
        else:
            dx, dy, dz = 0, 0, 2
            look_at_person(dx, dy, dz)

        # Draw x, y, z on frame
        cv2.putText(frame, "Estimated 3D Head Pos:", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"x: {dx:.2f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"y: {dy:.2f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"z: {dz:.2f}", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Convert frame to ImageTk for GUI
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb_frame)
        imgtk = ImageTk.PhotoImage(image=img)

        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)

        time.sleep(0.03)

# Start everything
update_animation()
random_blink_loop()
threading.Thread(target=face_tracking_loop, daemon=True).start()

# Run the GUI
root.mainloop()