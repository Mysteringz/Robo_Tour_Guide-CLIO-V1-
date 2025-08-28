import tkinter as tk
import random

class Eye:
    def __init__(self, canvas, index):
        self.canvas = canvas
        self.index = index
        self.center_x = 0
        self.center_y = 0
        self.eye_radius = 0
        self.pupil_radius = 0
        self.pupil = None
        self.current_dx = 0
        self.current_dy = 0
        self.target_dx = 0
        self.target_dy = 0

    def draw(self, center_x, center_y, size):
        self.center_x = center_x
        self.center_y = center_y
        self.eye_radius = size // 2
        self.pupil_radius = self.eye_radius // 2

        self.canvas.create_oval(center_x - self.eye_radius,
                                center_y - self.eye_radius,
                                center_x + self.eye_radius,
                                center_y + self.eye_radius,
                                fill='white', outline='black')

        self.pupil = self.canvas.create_oval(center_x - self.pupil_radius,
                                             center_y - self.pupil_radius,
                                             center_x + self.pupil_radius,
                                             center_y + self.pupil_radius,
                                             fill='black')

    def update_pupil_position(self):
        # Smooth move toward target
        self.current_dx += (self.target_dx - self.current_dx) * 0.2
        self.current_dy += (self.target_dy - self.current_dy) * 0.2
        self.canvas.coords(self.pupil,
                           self.center_x + self.current_dx - self.pupil_radius,
                           self.center_y + self.current_dy - self.pupil_radius,
                           self.center_x + self.current_dx + self.pupil_radius,
                           self.center_y + self.current_dy + self.pupil_radius)

    def look_at_3d_point(self, x, y, z):
        max_offset = self.eye_radius - self.pupil_radius - 2
        self.target_dx = max(-max_offset, min(max_offset, x))
        self.target_dy = max(-max_offset, min(max_offset, y))


class Eyes:
    def __init__(self, canvas):
        self.canvas = canvas
        self.eye_left = Eye(canvas, 0)
        self.eye_right = Eye(canvas, 1)

        # Fullscreen eyelid vars
        self.eyelid_position = 0.0
        self.top_lid = None
        self.bottom_lid = None
        self.blinking = False

    def draw(self, center_x, center_y, eye_spacing, eye_size):
        self.eye_left.draw(center_x - eye_spacing // 2, center_y, eye_size)
        self.eye_right.draw(center_x + eye_spacing // 2, center_y, eye_size)

    def update_pupils(self):
        self.eye_left.update_pupil_position()
        self.eye_right.update_pupil_position()

    def look_at(self, x, y, z):
        self.eye_left.look_at_3d_point(x, y, z)
        self.eye_right.look_at_3d_point(x, y, z)

    # ---------- Fullscreen eyelid ----------
    def _draw_eyelids(self):
        if self.top_lid:
            self.canvas.delete(self.top_lid)
        if self.bottom_lid:
            self.canvas.delete(self.bottom_lid)

        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        lid_height = int(h * self.eyelid_position / 2)

        self.top_lid = self.canvas.create_rectangle(0, 0, w, lid_height,
                                                    fill='black', outline='')
        self.bottom_lid = self.canvas.create_rectangle(0, h - lid_height, w, h,
                                                       fill='black', outline='')

    def _animate_blink(self, direction):
        step = 0.2
        self.eyelid_position += direction * step
        self.eyelid_position = max(0.0, min(1.0, self.eyelid_position))
        self._draw_eyelids()

        if direction == 1 and self.eyelid_position < 1.0:
            self.canvas.after(30, lambda: self._animate_blink(1))
        elif direction == -1 and self.eyelid_position > 0.0:
            self.canvas.after(30, lambda: self._animate_blink(-1))
        elif direction == 1:
            self.canvas.after(100, lambda: self._animate_blink(-1))
        else:
            self.blinking = False

    def blink(self):
        if not self.blinking:
            self.blinking = True
            self._animate_blink(1)


# Example usage
if __name__ == "__main__":
    root = tk.Tk()
    canvas = tk.Canvas(root, width=600, height=400, bg='grey')
    canvas.pack(fill="both", expand=True)

    eyes = Eyes(canvas)
    eyes.draw(300, 200, 150, 100)

    def update():
        eyes.update_pupils()
        root.after(30, update)

    def random_blink_loop():
        eyes.blink()
        root.after(random.randint(2000, 5000), random_blink_loop)

    update()
    random_blink_loop()

    root.mainloop()
