import tkinter as tk

def draw_eye(canvas, x, y, width, height, top_radius, bottom_depth, fill="white", outline="black"):
    """
    Draw a clean eye with a semi-circular top and concave bottom.
    """
    # Top semi-circle points
    top_points = [
        x, y + height / 2,
        x, y + top_radius,
        x + width / 2, y,
        x + width, y + top_radius,
        x + width, y + height / 2
    ]
    
    # Bottom concave curve points
    bottom_points = [
        x + width, y + height / 2,
        x + width, y + height / 2 + bottom_depth,
        x + width / 2, y + height,
        x, y + height / 2 + bottom_depth,
        x, y + height / 2
    ]
    
    # Combine points
    all_points = top_points + bottom_points
    
    # Draw smooth eye
    canvas.create_polygon(all_points, fill=fill, outline=outline, smooth=True)

# --- Tkinter setup ---
root = tk.Tk()
root.title("Nice Eye")

canvas = tk.Canvas(root, width=400, height=300, bg="lightblue")
canvas.pack()

# Draw an eye
draw_eye(canvas, x=100, y=100, width=200, height=100, top_radius=50, bottom_depth=30)

root.mainloop()
