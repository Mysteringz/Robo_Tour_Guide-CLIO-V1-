import tkinter as tk
import math

class HappyEyesApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Happy Eyes")

        # Parameters (controlled by sliders)
        self.eyes_factor   = tk.DoubleVar(value=1.25)   # overall eye size multiplier
        self.curve_depth   = tk.DoubleVar(value=127)    # bottom inward bump
        self.curve_width   = tk.DoubleVar(value=2)      # bottom bump spread
        self.top_arc       = tk.DoubleVar(value=127)    # top arc height
        self.top_rad       = tk.DoubleVar(value=52)     # top corner roundness
        self.bottom_fillet = tk.DoubleVar(value=0)      # bottom corner roundness
        self.arrow_fillet  = tk.DoubleVar(value=30)     # NEW – extra fillet at left arrow area
        self.eye_size = min(400 * 0.8, 1280 / 3 * 0.8)
        self.eye_gap  = 1280 * 0.16
        self.total_eye_width = self.eye_size * 2 + self.eye_gap
        self.start_x = (1280 - self.total_eye_width) / 2
        self.left_center_x  = self.start_x + self.eye_size / 2
        self.right_center_x = self.left_center_x + self.eye_size + self.eye_gap
        self.center_y = 400 / 2

        # Canvas
        self.canvas = tk.Canvas(root, width=1280, height=400,
                                bg="black", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # Sliders panel
        controls = tk.Frame(root, bg="black")
        controls.pack(fill="x")

        sliders = [
            ("Bottom Depth",  self.curve_depth,  -80, 200, 1),
            ("Bottom Width",  self.curve_width,    2,   6, 0.1),
            ("Top Arc",       self.top_arc,        0, 200, 1),
            ("Top Fillet",    self.top_rad,        0, 100, 1),
            ("Bottom Fillet", self.bottom_fillet,  0, 100, 1),
            ("Arrow Fillet",  self.arrow_fillet,   0, 100, 1),  # new slider
            ("Eye Size",      self.eyes_factor,  0.5,   2, 0.01)
        ]

        for label, var, lo, hi, step in sliders:
            tk.Label(controls, text=label, fg="white", bg="black").pack(side="left")
            tk.Scale(controls, from_=lo, to=hi, resolution=step, orient="horizontal",
                     variable=var, command=lambda e: self.redraw(),
                     bg="black", fg="cyan", troughcolor="gray",
                     length=120).pack(side="left")

        self.redraw()

    def draw_bean_eye(self, x, y, w, h,
                      bottom_depth, bottom_width,
                      top_arc_height, top_curve_radius,
                      bottom_fillet, arrow_fillet):
        eye_color = "#00FFFF"
        points = []

        # --- TOP ARC: left-to-right
        steps = 40
        fillet_ratio = top_curve_radius / w
        for i in range(steps + 1):
            t = i / steps
            t_adj = fillet_ratio + (1 - 2*fillet_ratio) * t
            xt = x - w/2 + t_adj * w
            yt = y - top_arc_height * math.sin(math.pi * t_adj)
            points.append((xt, yt))

        # --- RIGHT BOTTOM CORNER (clockwise)
        if bottom_fillet > 0:
            for a in range(0, 91, 10):   # quarter circle
                ang = math.radians(a)
                cx = (x + w/2 - bottom_fillet) + bottom_fillet * math.sin(ang)
                cy = (y + bottom_depth - bottom_fillet) + bottom_fillet * math.cos(ang)
                points.append((cx, cy))
        else:
            points.append((x + w/2, y + bottom_depth))

        # --- BOTTOM MIDLINE
        points.append((x, y + bottom_depth))

        # --- LEFT BOTTOM CORNER WITH ARROW FILLET
        if arrow_fillet > 0:
            for a in range(90, 181, 10):
                ang = math.radians(a)
                cx = (x - w/2 + arrow_fillet) + arrow_fillet * math.sin(ang)
                cy = (y + bottom_depth - arrow_fillet) + arrow_fillet * math.cos(ang)
                points.append((cx, cy))
        else:
            points.append((x - w/2, y + bottom_depth))

        flat = [c for p in points for c in p]
        self.canvas.create_polygon(flat, fill=eye_color,
                                   outline=eye_color, smooth=True)


    def redraw(self):
        self.canvas.delete("all")
        bd = self.curve_depth.get()
        bw = self.curve_width.get()
        ta = self.top_arc.get()
        tf = self.top_rad.get()
        bf = self.bottom_fillet.get()
        af = self.arrow_fillet.get()
        es = self.eyes_factor.get()

        self.draw_bean_eye(self.left_center_x,  self.center_y,
                           self.eye_size * es, self.eye_size * es,
                           bd, bw, ta, tf, bf, af)
        self.draw_bean_eye(self.right_center_x, self.center_y,
                           self.eye_size * es, self.eye_size * es,
                           bd, bw, ta, tf, bf, af)

if __name__ == "__main__":
    root = tk.Tk()
    app = HappyEyesApp(root)
    root.mainloop()
