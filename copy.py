import tkinter as tk
import math

class EyeConfig:
    """Python equivalent of the Arduino EyeConfig struct."""
    def __init__(self,
                 width=200, height=200,
                 offset_x=0, offset_y=0,
                 radius_top=8, radius_bottom=8,
                 slope_top=0.0, slope_bottom=0.0,
                 inverse_radius_top=0,
                 inverse_radius_bottom=0,
                 inverse_offset_top=0,
                 inverse_offset_bottom=0):
        self.Width = width
        self.Height = height
        self.OffsetX = offset_x
        self.OffsetY = offset_y
        self.Radius_Top = radius_top
        self.Radius_Bottom = radius_bottom
        self.Slope_Top = slope_top
        self.Slope_Bottom = slope_bottom
        self.Inverse_Radius_Top = inverse_radius_top
        self.Inverse_Radius_Bottom = inverse_radius_bottom
        self.Inverse_Offset_Top = inverse_offset_top
        self.Inverse_Offset_Bottom = inverse_offset_bottom


class EyeDrawer:
    @staticmethod
    def draw(canvas, center_x, center_y, config: EyeConfig, color="black"):
        # Compute slopes and corrected radii
        delta_y_top = config.Height * config.Slope_Top / 2.0
        delta_y_bottom = config.Height * config.Slope_Bottom / 2.0
        total_height = config.Height + delta_y_top - delta_y_bottom

        if (config.Radius_Bottom > 0 and config.Radius_Top > 0 and
            total_height - 1 < config.Radius_Bottom + config.Radius_Top):
            scale = (total_height - 1) / (config.Radius_Bottom + config.Radius_Top)
            config.Radius_Top *= scale
            config.Radius_Bottom *= scale

        TLc_y = center_y + config.OffsetY - config.Height/2 + config.Radius_Top - delta_y_top
        TLc_x = center_x + config.OffsetX - config.Width/2 + config.Radius_Top
        TRc_y = center_y + config.OffsetY - config.Height/2 + config.Radius_Top + delta_y_top
        TRc_x = center_x + config.OffsetX + config.Width/2 - config.Radius_Top
        BLc_y = center_y + config.OffsetY + config.Height/2 - config.Radius_Bottom - delta_y_bottom
        BLc_x = center_x + config.OffsetX - config.Width/2 + config.Radius_Bottom
        BRc_y = center_y + config.OffsetY + config.Height/2 - config.Radius_Bottom + delta_y_bottom
        BRc_x = center_x + config.OffsetX + config.Width/2 - config.Radius_Bottom

        EyeDrawer.fill_rectangle(canvas, TLc_x, TLc_y, BRc_x, BRc_y, color)
        EyeDrawer.fill_rectangle(canvas, TRc_x, TRc_y, BRc_x + config.Radius_Bottom, BRc_y, color)
        EyeDrawer.fill_rectangle(canvas, TLc_x - config.Radius_Top, TLc_y, BLc_x, BLc_y, color)
        EyeDrawer.fill_rectangle(canvas, TLc_x, TLc_y - config.Radius_Top, TRc_x, TRc_y, color)
        EyeDrawer.fill_rectangle(canvas, BLc_x, BLc_y, BRc_x, BRc_y + config.Radius_Bottom, color)

        EyeDrawer.fill_ellipse_corner(canvas, "tl", TLc_x, TLc_y,
                                      config.Radius_Top, config.Radius_Top, color)
        EyeDrawer.fill_ellipse_corner(canvas, "tr", TRc_x, TRc_y,
                                      config.Radius_Top, config.Radius_Top, color)
        EyeDrawer.fill_ellipse_corner(canvas, "bl", BLc_x, BLc_y,
                                      config.Radius_Bottom, config.Radius_Bottom, color)
        EyeDrawer.fill_ellipse_corner(canvas, "br", BRc_x, BRc_y,
                                      config.Radius_Bottom, config.Radius_Bottom, color)

    @staticmethod
    def fill_rectangle(canvas, x0, y0, x1, y1, color):
        canvas.create_rectangle(x0, y0, x1, y1, outline="", fill=color)

    @staticmethod
    def fill_ellipse_corner(canvas, corner, x0, y0, rx, ry, color):
        if rx <= 0 or ry <= 0:
            return
        bbox = (x0 - rx, y0 - ry, x0 + rx, y0 + ry)
        if corner == "tl":
            canvas.create_arc(bbox, start=90, extent=90, fill=color, outline="")
        elif corner == "tr":
            canvas.create_arc(bbox, start=0, extent=90, fill=color, outline="")
        elif corner == "bl":
            canvas.create_arc(bbox, start=180, extent=90, fill=color, outline="")
        elif corner == "br":
            canvas.create_arc(bbox, start=270, extent=90, fill=color, outline="")


# Preset equivalent to the C++ constant
Preset_Normal = EyeConfig(
    offset_x=0,
    offset_y=0,
    height=200,
    width=200,
    slope_top=5,
    slope_bottom=0,
    radius_top=30,
    radius_bottom=30,
    inverse_radius_top=0,
    inverse_radius_bottom=0,
    inverse_offset_top=0,
    inverse_offset_bottom=0
)

def main():
    root = tk.Tk()
    root.title("Two Eyes Demo")

    canvas_width, canvas_height = 1280, 400
    canvas = tk.Canvas(root, width=canvas_width, height=canvas_height, bg="white")
    canvas.pack()

    eyes_gap = 400  # distance between the centers of the two eyes
    center_y = canvas_height // 2
    center_x = canvas_width // 2

    # Calculate each eye's center x coordinate
    left_eye_x = center_x - eyes_gap/2
    right_eye_x = center_x + eyes_gap/2

    # Draw left and right eyes
    EyeDrawer.draw(canvas, left_eye_x, center_y, Preset_Normal, color="black")
    EyeDrawer.draw(canvas, right_eye_x, center_y, Preset_Normal, color="black")

    root.mainloop()

if __name__ == "__main__":
    main()
