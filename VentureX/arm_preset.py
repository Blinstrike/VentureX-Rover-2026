
import tkinter as tk
from tkinter import ttk, messagebox
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import json
import atexit
import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

# -------------------------------
# CONFIGURATION
# -------------------------------

SERVO_PINS = [17, 18, 27, 22]   # BCM GPIO numbers

MIN_ANGLE = -130
MAX_ANGLE = 130

INITIAL_ANGLE = 0
STEP = 1.0

# Adjust these to match your servo's pulse range
MIN_PULSE = 0.0005
MAX_PULSE = 0.0025

# -------------------------------
# GPIO INITIALIZATION
# -------------------------------

servos = []

for pin in SERVO_PINS:
    servo = AngularServo(
        pin,
        min_angle=MIN_ANGLE,
        max_angle=MAX_ANGLE,
        min_pulse_width=MIN_PULSE,
        max_pulse_width=MAX_PULSE,
        initial_angle=INITIAL_ANGLE
    )
    servos.append(servo)


def cleanup():
    for servo in servos:
        servo.detach()
        servo.close()


atexit.register(cleanup)

# -------------------------------
# STATE
# -------------------------------

angles = [INITIAL_ANGLE] * len(SERVO_PINS)
positions = {}

# -------------------------------
# GUI
# -------------------------------

root = tk.Tk()
root.title("Robotic Arm - Servo Calibration")
root.geometry("320x240")

step_var = tk.DoubleVar(value=STEP)

angle_vars = []
angle_labels = []
angle_entries = []

main = ttk.Frame(root, padding=15)
main.pack(fill="both", expand=True)

ttk.Label(
    main,
    text="ROBOTIC ARM CALIBRATION",
    font=("Arial", 19, "bold")
).pack(pady=10)

# -------------------------------
# SERVO CONTROL
# -------------------------------

step_frame = ttk.LabelFrame(
    main,
    text="Adjustment precision",
    padding=10
)
step_frame.pack(fill="x", pady=10)

ttk.Label(
    step_frame,
    text="Step size (degrees):"
).pack(side="left")

ttk.Entry(
    step_frame,
    textvariable=step_var,
    width=10
).pack(side="left", padx=10)


def update_servo(index, value):
    value = max(MIN_ANGLE, min(MAX_ANGLE, float(value)))

    angles[index] = round(value, 2)

    # Move physical servo
    servos[index].angle = angles[index]

    angle_vars[index].set(angles[index])

    angle_labels[index].config(
        text=f"{angles[index]:.2f}°"
    )


def adjust_servo(index, direction):
    try:
        step = float(step_var.get())

        if step <= 0:
            raise ValueError

        update_servo(
            index,
            angles[index] + direction * step
        )

    except ValueError:
        messagebox.showerror(
            "Invalid step",
            "Enter a positive number."
        )


servo_frame = ttk.LabelFrame(
    main,
    text="Individual servo controls",
    padding=10
)
servo_frame.pack(fill="x", pady=10)

for i, pin in enumerate(SERVO_PINS):

    frame = ttk.LabelFrame(
        servo_frame,
        text=f"Servo {i + 1} | GPIO {pin}",
        padding=8
    )
    frame.pack(fill="x", pady=5)

    angle_var = tk.DoubleVar(value=INITIAL_ANGLE)
    angle_vars.append(angle_var)

    ttk.Button(
        frame,
        text="−",
        width=4,
        command=lambda i=i: adjust_servo(i, -1)
    ).pack(side="left")

    slider = ttk.Scale(
        frame,
        from_=MIN_ANGLE,
        to=MAX_ANGLE,
        variable=angle_var,
        command=lambda value, i=i: update_servo(i, value)
    )
    slider.pack(side="left", fill="x", expand=True, padx=10)

    label = ttk.Label(
        frame,
        text="0.00°",
        width=9
    )
    label.pack(side="left")

    angle_labels.append(label)

    ttk.Button(
        frame,
        text="+",
        width=4,
        command=lambda i=i: adjust_servo(i, 1)
    ).pack(side="left", padx=5)

    entry = ttk.Entry(frame, width=8)
    entry.insert(0, str(INITIAL_ANGLE))
    entry.pack(side="left")

    angle_entries.append(entry)

    ttk.Button(
        frame,
        text="Set",
        command=lambda i=i: set_manual(i)
    ).pack(side="left", padx=5)


def set_manual(index):
    try:
        value = float(angle_entries[index].get())
        update_servo(index, value)
    except ValueError:
        messagebox.showerror("Error", "Invalid angle")


# -------------------------------
# POSITION MANAGEMENT
# -------------------------------

position_frame = ttk.LabelFrame(
    main,
    text="Saved positions",
    padding=10
)
position_frame.pack(fill="both", expand=True, pady=10)

name_var = tk.StringVar()

ttk.Entry(
    position_frame,
    textvariable=name_var
).pack(fill="both", pady=5)

position_list = tk.Listbox(
    position_frame,
    height=5
)
position_list.pack(fill="both", pady=5)


def save_position():
    name = name_var.get().strip()

    if not name:
        messagebox.showerror("Error", "Enter a name")
        return

    positions[name] = angles.copy()

    position_list.delete(0, tk.END)

    for position in positions:
        position_list.insert(tk.END, position)


def load_position():
    selection = position_list.curselection()

    if not selection:
        return

    name = position_list.get(selection[0])

    for i, angle in enumerate(positions[name]):
        update_servo(i, angle)


def export_positions():
    with open("arm_positions.json", "w") as file:
        json.dump(positions, file, indent=4)

    messagebox.showinfo(
        "Export",
        "Positions exported to arm_positions.json"
    )


def reset_position():
    for i in range(len(servos)):
        update_servo(i, INITIAL_ANGLE)


buttons = ttk.Frame(position_frame)
buttons.pack(fill="both", pady=5)

ttk.Button(
    buttons,
    text="Save",
    command=save_position
).pack(side="left", padx=5)

ttk.Button(
    buttons,
    text="Load",
    command=load_position
).pack(side="left", padx=5)

ttk.Button(
    buttons,
    text="Export JSON",
    command=export_positions
).pack(side="left", padx=5)

ttk.Button(
    buttons,
    text="Reset",
    command=reset_position
).pack(side="left", padx=5)

root.mainloop()
