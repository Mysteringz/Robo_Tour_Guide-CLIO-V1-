import tkinter as tk
import math
import random

class Eye:
    def __init__(self, canvas):
        self.canvas = canvas
        self.eye = None
        self.pupil = None
        self.eyelid = None
        self.brow_mask = None
        self.eye_size = 0
        self.center_x = 0
        self.center_y = 0
        self.eye_radius = 0
        self.pupil_radius = 0
        self.pupil_move_radius = 0

    def draw(self, center_x, center_y, eye_size):
        self.center_x = center_x
        self.center_y = center_y
        self.eye_size = eye_size
        self.eye_radius = eye_size // 2
        self.pupil_radius = eye_size // 6
        self.pupil_move_radius = self.eye_radius - self.pupil_radius

        for shape in [self.eye, self.pupil, self.eyelid, self.brow_mask]:
            if shape:
                self.canvas.delete(shape)

        self.eye = self.canvas.create_oval(center_x - self.eye_radius,
                                           center_y - self.eye_radius,
                                           center_x + self.eye_radius,
                                           center_y + self.eye_radius,
                                           fill='white', outline='black', width=3)

        self.pupil = self.canvas.create_oval(center_x - self.pupil_radius,
                                             center_y - self.pupil_radius,
                                             center_x + self.pupil_radius,
                                             center_y + self.pupil_radius,
                                             fill='black')

        self.eyelid = self.canvas.create_rectangle(center_x - self.eye_radius,
                                                   center_y - self.eye_radius,
                                                   center_x + self.eye_radius,
                                                   center_y - self.eye_radius,
                                                   fill='black', outline='black')

        self.brow_mask = self.canvas.create_polygon(0, 0, 0, 0, 0, 0, fill='', outline='')

    def move_pupil(self, mouse_x, mouse_y):
        dx = mouse_x - self.center_x
        dy = mouse_y - self.center_y
        distance = math.hypot(dx, dy)

        if distance > self.pupil_move_radius:
            dx = dx / distance * self.pupil_move_radius
            dy = dy / distance * self.pupil_move_radius

        new_x = self.center_x + dx
        new_y = self.center_y + dy
        self.canvas.coords(self.pupil,
                           new_x - self.pupil_radius, new_y - self.pupil_radius,
                           new_x + self.pupil_radius, new_y + self.pupil_radius)

    def set_eyelid_position(self, height):
        self.canvas.coords(self.eyelid,
                           self.center_x - self.eye_radius,
                           self.center_y - self.eye_radius,
                           self.center_x + self.eye_radius,
                           self.center_y - self.eye_radius + height)

    def set_brow_mask(self, shape):
        size = self.eye_radius
        cx, cy = self.center_x, self.center_y

        if shape == 'angry':
            if self == eyes[0]:  # Left eye: inward tilt "/"
                points = [
                    cx + size, cy - size,
                    cx + size * 0.1, cy - size - size * 0.1,
                    cx + size, cy
                ]
            else:  # Right eye: inward tilt "\"
                points = [
                    cx - size, cy - size,
                    cx - size * 0.1, cy - size - size * 0.1,
                    cx - size, cy
                ]
        elif shape == 'funny':
            # Draws big top lid as if laughing
            points = [
                cx - size, cy - size,
                cx + size, cy - size,
                cx + size, cy - size + size * 0.3,
                cx - size, cy - size + size * 0.3
            ]
        elif shape == 'sleepy':
            points = [
                cx - size, cy,
                cx + size, cy,
                cx + size, cy + size * -1,
                cx - size, cy + size * -1
            ]
        elif shape == 'surprised':
            # Hide mask, pupil will change
            points = [0, 0, 0, 0, 0, 0]
        else:
            points = [0, 0, 0, 0, 0, 0]

        if shape == 'surprised':
            self.canvas.itemconfig(self.pupil, fill='red')
        else:
            self.canvas.itemconfig(self.pupil, fill='black')

        self.canvas.coords(self.brow_mask, *points)
        self.canvas.itemconfig(self.brow_mask, fill='black', outline='black' if shape != 'surprised' else '')

    def reset_brow(self):
        self.canvas.coords(self.brow_mask, 0, 0, 0, 0, 0, 0)
        self.canvas.itemconfig(self.brow_mask, fill='', outline='')
        self.canvas.itemconfig(self.pupil, fill='black')


# === Setup ===
def on_resize(event):
    canvas_width = event.width
    canvas_height = event.height
    canvas.delete("all")

    eye_size = int(min(canvas_height * 0.8, canvas_width / 3 * 0.8))
    eye_gap = canvas_width * 0.1
    total_eye_width = eye_size * 2 + eye_gap
    start_x = (canvas_width - total_eye_width) / 2

    left_center_x = start_x + eye_size / 2
    right_center_x = left_center_x + eye_size + eye_gap
    center_y = canvas_height / 2

    eyes[0].draw(left_center_x, center_y, eye_size)
    eyes[1].draw(right_center_x, center_y, eye_size)


def on_mouse_move(event):
    for eye in eyes:
        eye.move_pupil(event.x, event.y)


# === Emotion Handling ===
active_emotion = None
emotion_keys = {
    '1': 'angry',
    '2': 'funny',
    '3': 'sleepy',
    '4': 'surprised'
}


def apply_emotion():
    if active_emotion:
        for eye in eyes:
            eye.set_brow_mask(active_emotion)
    else:
        for eye in eyes:
            eye.reset_brow()

    root.after(100, apply_emotion)


def on_key_press(event):
    global active_emotion
    if event.char in emotion_keys:
        active_emotion = emotion_keys[event.char]


def on_key_release(event):
    global active_emotion
    if event.char in emotion_keys and active_emotion == emotion_keys[event.char]:
        active_emotion = None


# === Blink ===
def blink_animation(step=0, closing=True):
    steps = 5
    max_height = eyes[0].eye_size

    if closing:
        height = max_height / steps * step
        if step > steps:
            root.after(30, lambda: blink_animation(0, closing=False))
            return
    else:
        height = max_height - (max_height / steps * step)
        if step > steps:
            schedule_next_blink()
            return

    for eye in eyes:
        eye.set_eyelid_position(height)

    root.after(30, lambda: blink_animation(step + 1, closing))


def schedule_next_blink():
    delay = random.randint(2000, 4000)
    root.after(delay, lambda: blink_animation())

# === GUI Init ===
root = tk.Tk()
root.title("2D Eyes with Emotions")

canvas = tk.Canvas(root, width=1280, height=400, bg='black')
canvas.pack(fill=tk.BOTH, expand=True)

eyes = [Eye(canvas), Eye(canvas)]

canvas.bind('<Configure>', on_resize)
canvas.bind('<Motion>', on_mouse_move)
root.bind('<KeyPress>', on_key_press)
root.bind('<KeyRelease>', on_key_release)

schedule_next_blink()
apply_emotion()

root.mainloop()
