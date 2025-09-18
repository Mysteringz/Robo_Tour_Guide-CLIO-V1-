from tkinter import *

def update_polygon(val):
    # Clear the canvas
    canvas.delete("all")
    # Get the current value of radius from the slider
    radius = int(val)
    # Define the new points based on the updated radius
    x1, y1 = 30, 30
    x2, y2 = 230, 230
    x3, y3 = 630, 230
    x4, y4 = 830, 30
    points = (
        (x1, y1),           #1
        (x1, y1),           #2
        (x2, y2),           #3
        (x2, y2),           #4
        (x3, y3),           #5
        (x3, y3),           #6
        (x4, y4),           #7
        (x4, y4),           #8
        (x4, y4 + radius),  #9
        (x4, y4),           #10
        (x4 - radius, y4),  #11
        (x4 - radius, y4),  #12
    )
    # Draw the polygon
    canvas.create_polygon(points, fill="red", smooth=1)
    # Add text labels for the points
    for i, (x, y) in enumerate(points):
        canvas.create_text(x, y, text=f"{x}, {y} #{i+1:02}")

# Create the main window
root = Tk()
canvas = Canvas(root, width=865, height=650, bg="white")
canvas.pack()
# Initialize radius and create the slider
radius_slider = Scale(root, to=800, orient=HORIZONTAL, length=865, command=update_polygon)
radius_slider.pack()
# Initial call to draw the polygon with the initial radius
update_polygon(radius_slider.get())
# Bind the Return key to save the canvas as an EPS file
root.bind("<Return>", lambda a: canvas.postscript(file="test15.eps"))
# Start the Tkinter main loop
root.mainloop()