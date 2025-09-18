import tkinter as tk
import math

class HappyEyesApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Happy Eyes")

        # Parameters (controlled by sliders)
        self.eyes_factor = tk.DoubleVar(value=1.25)      # overall eye size multiplier
        self.curve_depth = tk.DoubleVar(value=127)    # bottom inward bump
        self.curve_width = tk.DoubleVar(value=2)     # bottom bump spread
        self.top_arc = tk.DoubleVar(value=127)        # top arc height
        self.top_rad = tk.DoubleVar(value=52)     # top corner roundness
        self.bottom_fillet = tk.DoubleVar(value=0)  # bottom corner roundness
        self.eye_size = min(400 * 0.8, 1280 / 3 * 0.8)
        self.eye_gap = 1280 * 0.16
        self.total_eye_width = self.eye_size * 2 + self.eye_gap
        self.start_x = (1280 - self.total_eye_width) / 2
        self.left_center_x = self.start_x + self.eye_size / 2
        self.right_center_x = self.left_center_x + self.eye_size + self.eye_gap
        self.center_y = 400 / 2

        # Canvas
        self.canvas = tk.Canvas(root, width=1280, height=400, bg="black", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # Sliders panel
        controls = tk.Frame(root, bg="black")
        controls.pack(fill="x")

        sliders = [
            ("Bottom Depth", self.curve_depth, -80, 200, 1),
            ("Bottom Width", self.curve_width, 2, 6, 0.1),
            ("Top Arc", self.top_arc, 0, 200, 1),
            ("Top Fillet", self.top_rad, 0, 100, 1),
            ("Bottom Fillet", self.bottom_fillet, 0, 100, 1),
            ("Eye Size", self.eyes_factor, 0.5, 2, 0.01)
        ]

        for label, var, lo, hi, step in sliders:
            tk.Label(controls, text=label, fg="white", bg="black").pack(side="left")
            tk.Scale(controls, from_=lo, to=hi, resolution=step, orient="horizontal",
                     variable=var, command=lambda e: self.redraw(),
                     bg="black", fg="cyan", troughcolor="gray", length=120).pack(side="left")

        # Initial draw
        self.redraw()

    def draw_bean_eye(self, x, y, w, h, bottom_depth, bottom_width, top_arc_height, top_curve_radius, bottom_fillet):
        eye_color = "#00FFFF"
        points = []

        # Top arc: generate smooth curve with filleted ends
        steps = 20
        for i in range(steps + 1):
            t = i / steps
            # apply fillet: shrink the start/end t values
            fillet_ratio = top_curve_radius / w
            t_adjusted = fillet_ratio + (1 - 2*fillet_ratio) * t
            xt = x - w/2 + t_adjusted * w
            # Sine curve for top arc
            yt = y - top_arc_height * math.sin(math.pi * t_adjusted)
            points.append((xt, yt))

        # Bottom concave: simple 3 points
        points += [
            (x + w/2 - bottom_fillet, y + bottom_depth),
            (x, y + bottom_depth/2),
            (x - w/2 + bottom_fillet, y + bottom_depth)
        ]

        flat_points = [coord for point in points for coord in point]
        self.canvas.create_polygon(flat_points, fill=eye_color, outline=eye_color, smooth=True)

    def redraw(self):
        """Redraw both eyes with current slider values."""
        self.canvas.delete("all")
        bd = self.curve_depth.get()
        bw = self.curve_width.get()
        ta = self.top_arc.get()
        tf = self.top_rad.get()
        bf = self.bottom_fillet.get()
        es = self.eyes_factor.get()

        # Left and right eyes
        self.draw_bean_eye(self.left_center_x, self.center_y, self.eye_size * es, self.eye_size * es , bd, bw, ta, tf, bf)
        self.draw_bean_eye(self.right_center_x, self.center_y, self.eye_size * es, self.eye_size * es , bd, bw, ta, tf, bf)


if __name__ == "__main__":
    root = tk.Tk()
    app = HappyEyesApp(root)
    root.mainloop()
