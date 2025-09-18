import tkinter as tk

class CrescentEyesApp:
    def __init__(self, root):
        self.root = root
        root.title("Crescent Happy Eyes")

        self.canvas = tk.Canvas(root, width=800, height=400, bg="#F0F0F0")
        self.canvas.pack(fill="both", expand=True)

        controls = tk.Frame(root)
        controls.pack(fill="x", padx=10, pady=6)

        tk.Label(controls, text="Crescent fillet (0-100):").pack(side="left")
        self.fillet = tk.IntVar(value=40)  # controls inner offset -> crescent thickness
        fillet_slider = tk.Scale(controls, from_=0, to=100, orient="horizontal",
                                 variable=self.fillet, command=lambda e: self.redraw())
        fillet_slider.pack(side="left", fill="x", expand=True, padx=6)

        # Eye size slider
        tk.Label(controls, text="Eye size:").pack(side="left", padx=(10,0))
        self.eye_size = tk.IntVar(value=220)
        size_slider = tk.Scale(controls, from_=80, to=350, orient="horizontal",
                               variable=self.eye_size, command=lambda e: self.redraw())
        size_slider.pack(side="left", padx=6)

        # Bind resize so drawing stays centered
        self.canvas.bind("<Configure>", lambda e: self.redraw())

        self.redraw()

    def draw_crescent(self, cx, cy, w, h, fillet, color="#222222"):
        """
        Draw a crescent by drawing a big filled oval (outer) then overlaying a smaller
        offset oval with the background color to "cut out" the inner part.
        cx,cy center of the outer oval; w,h width/height of outer oval.
        fillet: 0..100 makes the inner oval larger/smaller -> affects crescent thickness.
        """
        bg = self.canvas["bg"]
        # Outer oval bounds
        x0 = cx - w/2
        y0 = cy - h/2
        x1 = cx + w/2
        y1 = cy + h/2
        # Draw outer shape
        outer = self.canvas.create_oval(x0, y0, x1, y1, fill=color, outline="")

        # inner offset depends on fillet; offset the inner oval downward so the crescent opens upward (happy)
        # fillet 0 -> inner same as outer (no crescent); fillet 100 -> very thin crescent
        f = fillet / 100.0
        # scale inner oval relative to outer
        inner_w = w * (0.5 + 0.45 * f)   # between 0.5w and 0.95w
        inner_h = h * (0.5 + 0.45 * f)
        # vertical offset to make crescent "smile" shaped - positive -> inner shifted down
        vert_offset = h * (0.15 + 0.25 * (1 - f))

        ix0 = cx - inner_w/2
        iy0 = cy - inner_h/2 + vert_offset
        ix1 = cx + inner_w/2
        iy1 = cy + inner_h/2 + vert_offset

        # Draw inner oval using background color to remove area -> crescent
        inner = self.canvas.create_oval(ix0, iy0, ix1, iy1, fill=bg, outline="")

        # rounded ends: draw small circles to smooth corners if needed
        # We'll draw two small circles at the tips of the crescent to ensure rounded edges
        tip_radius = max(6, int(min(w,h) * 0.045))
        # left tip position approx at left-most of outer but slightly up
        left_tip_x = x0 + tip_radius
        left_tip_y = (y0 + y1)/2 - h*0.15
        right_tip_x = x1 - tip_radius
        right_tip_y = (y0 + y1)/2 - h*0.15
        self.canvas.create_oval(left_tip_x-tip_radius, left_tip_y-tip_radius,
                                left_tip_x+tip_radius, left_tip_y+tip_radius,
                                fill=color, outline="")
        self.canvas.create_oval(right_tip_x-tip_radius, right_tip_y-tip_radius,
                                right_tip_x+tip_radius, right_tip_y+tip_radius,
                                fill=color, outline="")

        return outer, inner

    def draw_pupil(self, cx, cy, size, color="#FFFFFF"):
        """Small highlight / pupil inside crescent (optional)"""
        r = size * 0.12
        return self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=color, outline="")

    def redraw(self):
        self.canvas.delete("all")
        W = self.canvas.winfo_width()
        H = self.canvas.winfo_height()

        # parameters
        eye_w = self.eye_size.get()
        eye_h = eye_w * 0.6
        spacing = eye_w * 0.35
        fillet = self.fillet.get()

        center_y = H * 0.45
        left_cx = W/2 - (eye_w/2 + spacing/2)
        right_cx = W/2 + (eye_w/2 + spacing/2)

        # draw left and right crescents (dark)
        left_eye_shapes = self.draw_crescent(left_cx, center_y, eye_w, eye_h, fillet, color="#222222")
        right_eye_shapes = self.draw_crescent(right_cx, center_y, eye_w, eye_h, fillet, color="#222222")

        # add tiny glints (white) to give "happy" sparkle — positioned near top-left of each crescent
        glint_r = max(4, int(eye_w * 0.06))

        # Optional: cute smiling mouth to emphasize "happy"
        mouth_w = eye_w * 1.2
        mouth_h = eye_h * 0.5
        mx0 = W/2 - mouth_w/2
        my0 = center_y + eye_h*0.6
        mx1 = W/2 + mouth_w/2
        my1 = my0 + mouth_h
        # thicker arc-like smile using an oval and then masking
        self.canvas.create_arc(mx0, my0, mx1, my1, start=0, extent=180, style="arc",
                               width=max(6, int(eye_w*0.06)), outline="#222222")

if __name__ == "__main__":
    root = tk.Tk()
    app = CrescentEyesApp(root)
    root.mainloop()
