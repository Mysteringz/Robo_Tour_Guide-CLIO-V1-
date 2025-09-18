import tkinter as tk
from PIL import Image, ImageTk

# Main window
root = tk.Tk()
root.title("Image Viewer")

# Load image
try:
    img = Image.open("ART.png")
except FileNotFoundError:
    print("Error: ART.png not found in the current directory.")
    exit()

# Convert image for Tkinter
img_tk = ImageTk.PhotoImage(img)

# Create canvas and display image
canvas = tk.Canvas(root, width=img.width, height=img.height, bg="gray")
canvas.pack()
canvas.create_image(0, 0, anchor="nw", image=img_tk)

root.mainloop()
