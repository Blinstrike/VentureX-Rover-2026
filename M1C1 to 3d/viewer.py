
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import threading
import queue
import time

import matplotlib

matplotlib.use("TkAgg")

import matplotlib.pyplot as plt

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from m1c1 import M1C1Lidar
from servo import ServoController
from scanner3d import Scanner3D


# ============================================================
# CONFIGURATION
# ============================================================

LIDAR_PORT = "/dev/serial0"
LIDAR_BAUDRATE = 115200

SERVO_PIN = 17

DEFAULT_ANGLE_STEP = 2.0
DEFAULT_POINT_LIFETIME = 30.0

SERVO_SETTLE_TIME = 0.15

PLOT_REFRESH_MS = 100


# ============================================================
# VIEWER
# ============================================================

class LidarViewer:

    def __init__(self, root, scanner, lidar, servo):

        self.root = root
        self.scanner = scanner
        self.lidar = lidar
        self.servo = servo

        self.root.title("M1C1 Mini - 3D LiDAR Viewer")
        self.root.geometry("1200x850")

        self.points = []

        # Each entry: (x, y, z, timestamp)

        self.data_queue = queue.Queue()

        self.stop_event = threading.Event()
        self.reset_event = threading.Event()
        self.pause_event = threading.Event()

        self.running = True

        self.dark_mode = tk.BooleanVar(value=True)

        self.lifetime_var = tk.StringVar(
            value=str(DEFAULT_POINT_LIFETIME)
        )

        self.step_var = tk.StringVar(
            value=str(DEFAULT_ANGLE_STEP)
        )

        self.status_var = tk.StringVar(
            value="Initializing..."
        )

        self.angle_var = tk.StringVar(
            value="Servo: 0°"
        )

        self.count_var = tk.StringVar(
            value="Points: 0"
        )

        self._build_ui()
        self._apply_theme()

        # Move servo to starting position BEFORE scanning.
        self.status_var.set("Moving servo to 0 degrees...")

        self.servo.set_angle(-90, settle_time=1.0)

        self.worker = threading.Thread(
            target=self._scan_worker,
            daemon=True
        )

        self.worker.start()

        self.root.after(
            PLOT_REFRESH_MS,
            self._update_plot
        )

        self.root.protocol(
            "WM_DELETE_WINDOW",
            self.close
        )

    # ========================================================
    # INTERFACE
    # ========================================================

    def _build_ui(self):

        controls = ttk.Frame(self.root, padding=8)
        controls.pack(fill=tk.X)

        # Scan controls

        ttk.Button(
            controls,
            text="Pause / Resume",
            command=self.toggle_pause
        ).pack(side=tk.LEFT, padx=4)

        ttk.Button(
            controls,
            text="Reset & Restart",
            command=self.reset_scan
        ).pack(side=tk.LEFT, padx=4)

        ttk.Separator(
            controls,
            orient=tk.VERTICAL
        ).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        # Servo angle step

        ttk.Label(
            controls,
            text="Servo step (°):"
        ).pack(side=tk.LEFT, padx=4)

        ttk.Entry(
            controls,
            textvariable=self.step_var,
            width=7
        ).pack(side=tk.LEFT)

        ttk.Button(
            controls,
            text="Apply",
            command=self.apply_step
        ).pack(side=tk.LEFT, padx=4)

        # Point lifetime

        ttk.Label(
            controls,
            text="Point lifetime (s):"
        ).pack(side=tk.LEFT, padx=(12, 4))

        ttk.Entry(
            controls,
            textvariable=self.lifetime_var,
            width=7
        ).pack(side=tk.LEFT)

        # Export

        ttk.Button(
            controls,
            text="Export PLY",
            command=self.export_ply
        ).pack(side=tk.RIGHT, padx=4)

        # Dark mode

        ttk.Checkbutton(
            controls,
            text="Dark mode",
            variable=self.dark_mode,
            command=self._apply_theme
        ).pack(side=tk.RIGHT, padx=10)

        # Plot

        self.fig = plt.Figure(
            figsize=(9, 7),
            dpi=100
        )

        self.ax = self.fig.add_subplot(
            111,
            projection="3d"
        )

        self.scatter = self.ax.scatter(
            [], [], [],
            s=1,
            c=[],
            cmap="viridis"
        )

        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")

        self.ax.set_title("M1C1 3D Point Cloud")

        self.ax.set_xlim(-6000, 6000)
        self.ax.set_ylim(-6000, 6000)
        self.ax.set_zlim(-6000, 6000)

        self.ax.set_box_aspect((1, 1, 1))

        self.canvas = FigureCanvasTkAgg(
            self.fig,
            master=self.root
        )

        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH,
            expand=True
        )

        # Status bar

        status = ttk.Frame(self.root, padding=8)
        status.pack(fill=tk.X)

        ttk.Label(
            status,
            textvariable=self.status_var
        ).pack(side=tk.LEFT)

        ttk.Label(
            status,
            textvariable=self.angle_var
        ).pack(side=tk.RIGHT, padx=15)

        ttk.Label(
            status,
            textvariable=self.count_var
        ).pack(side=tk.RIGHT, padx=15)

    # ========================================================
    # THEME
    # ========================================================

    def _apply_theme(self):

        dark = self.dark_mode.get()

        if dark:
            bg = "#181818"
            fg = "#eeeeee"
            axes_bg = "#202020"
            grid = "#555555"
        else:
            bg = "#f0f0f0"
            fg = "#222222"
            axes_bg = "#ffffff"
            grid = "#cccccc"

        self.root.configure(bg=bg)

        self.fig.patch.set_facecolor(bg)
        self.ax.set_facecolor(axes_bg)

        self.ax.xaxis.label.set_color(fg)
        self.ax.yaxis.label.set_color(fg)
        self.ax.zaxis.label.set_color(fg)

        self.ax.title.set_color(fg)

        for axis in (
            self.ax.xaxis,
            self.ax.yaxis,
            self.ax.zaxis
        ):
            axis.set_tick_params(colors=fg)
            axis.pane.set_facecolor(axes_bg)

        self.ax.grid(True, color=grid)

        for spine in self.ax.spines.values():
            spine.set_color(fg)

        self.canvas.draw_idle()

    # ========================================================
    # SCANNING THREAD
    # ========================================================

    def _scan_worker(self):

        angle_index = 0

        while not self.stop_event.is_set():

            # Reset requested

            if self.reset_event.is_set():

                try:
                    self.scanner.reset()
                except Exception as e:
                    self.data_queue.put(
                        ("error", str(e))
                    )
                    break

                angle_index = 0

                self.reset_event.clear()

                self.data_queue.put(
                    ("status", "Restarting scan...")
                )

            # Pause

            if self.pause_event.is_set():

                self.data_queue.put(
                    ("status", "Scanning paused")
                )

                time.sleep(0.1)
                continue

            try:

                angles = self.scanner.get_servo_angles()

                if angle_index >= len(angles):
                    angle_index = 0

                angle = angles[angle_index]

                self.data_queue.put(
                    ("angle", angle)
                )

                points = self.scanner.scan_position(
                    angle
                )

                timestamp = time.monotonic()

                self.data_queue.put(
                    ("points", (points, timestamp))
                )

                self.data_queue.put(
                    ("status", "Scanning")
                )

                angle_index += 1

                if angle_index >= len(angles):
                    angle_index = 0

            except TimeoutError:

                self.data_queue.put(
                    ("status", "LiDAR timeout - retrying")
                )

            except Exception as e:

                self.data_queue.put(
                    ("error", str(e))
                )

                break

    # ========================================================
    # CONTROLS
    # ========================================================

    def toggle_pause(self):

        if self.pause_event.is_set():
            self.pause_event.clear()
            self.status_var.set("Resuming...")
        else:
            self.pause_event.set()

    def reset_scan(self):

        self.points.clear()

        self._clear_queue()

        self.reset_event.set()

        self.status_var.set(
            "Returning servo to 0°..."
        )

        self._update_plot()

    def apply_step(self):

        try:

            step = float(self.step_var.get())

            if not 0 < step <= 180:
                raise ValueError

            self.scanner.set_angle_step(step)

            self.status_var.set(
                f"Servo step updated to {step}°"
            )

        except ValueError:

            messagebox.showerror(
                "Invalid angle step",
                "Enter a number greater than 0 and at most 180."
            )

    def _clear_queue(self):

        while True:
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break

    # ========================================================
    # POINT MANAGEMENT
    # ========================================================

    def _update_plot(self):

        if not self.running:
            return

        # Process incoming scan data.

        while True:

            try:
                kind, data = self.data_queue.get_nowait()

            except queue.Empty:
                break

            if kind == "points":

                points, timestamp = data

                for x, y, z in points:
                    self.points.append(
                        (x, y, z, timestamp)
                    )

            elif kind == "angle":

                self.angle_var.set(
                f"Servo: {data:+.1f}°"
                )

            elif kind == "status":

                self.status_var.set(data)

            elif kind == "error":

                self.status_var.set(
                    f"Error: {data}"
                )

        # Remove expired points.

        try:
            lifetime = max(
                0.0,
                float(self.lifetime_var.get())
            )
        except ValueError:
            lifetime = DEFAULT_POINT_LIFETIME

        now = time.monotonic()

        if lifetime == 0:
            self.points.clear()
        else:
            self.points = [
                p for p in self.points
                if now - p[3] <= lifetime
            ]

        # Update scatter plot.

        count = len(self.points)

        self.count_var.set(
            f"Points: {count:,}"
        )

        if count:

            xs = [p[0] for p in self.points]
            ys = [p[1] for p in self.points]
            zs = [p[2] for p in self.points]

            self.scatter._offsets3d = (
                xs, ys, zs
            )

            self.scatter.set_array(
                __import__("numpy").array(zs)
            )

        else:

            self.scatter._offsets3d = (
                [], [], []
            )

            self.scatter.set_array(None)

        self.canvas.draw_idle()

        self.root.after(
            PLOT_REFRESH_MS,
            self._update_plot
        )

    # ========================================================
    # PLY EXPORT
    # ========================================================

    def export_ply(self):

        try:
            lifetime = max(
                0.0,
                float(self.lifetime_var.get())
            )
        except ValueError:
            messagebox.showerror(
                "Invalid lifetime",
                "Enter a valid point lifetime first."
            )
            return

        now = time.monotonic()

        # Export exactly the points currently visible.

        if lifetime == 0:
            visible = []
        else:
            visible = [
                p for p in self.points
                if now - p[3] <= lifetime
            ]

        if not visible:

            messagebox.showinfo(
                "Export",
                "There are no visible points to export."
            )

            return

        filename = filedialog.asksaveasfilename(
            defaultextension=".ply",
            filetypes=[
                ("PLY point cloud", "*.ply")
            ],
            initialfile="m1c1_pointcloud.ply"
        )

        if not filename:
            return

        try:

            with open(
                filename,
                "w",
                encoding="ascii"
            ) as f:

                f.write("ply\n")
                f.write("format ascii 1.0\n")

                f.write(
                    f"element vertex {len(visible)}\n"
                )

                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("end_header\n")

                for x, y, z, timestamp in visible:

                    f.write(
                        f"{x:.4f} {y:.4f} {z:.4f}\n"
                    )

            messagebox.showinfo(
                "Export successful",
                f"Exported {len(visible):,} points."
            )

        except OSError as e:

            messagebox.showerror(
                "Export error",
                str(e)
            )

    # ========================================================
    # CLEANUP
    # ========================================================

    def close(self):

        self.running = False

        self.stop_event.set()
        self.reset_event.clear()

        self.status_var.set(
            "Stopping scanner..."
        )

        def finish():

            self.lidar.close()
            self.servo.cleanup()

            self.root.destroy()

        def wait_for_worker():

            if self.worker.is_alive():

                self.root.after(
                    100,
                    wait_for_worker
                )

            else:
                finish()

        wait_for_worker()


# ============================================================
# MAIN
# ============================================================

def main():

    lidar = None
    servo = None

    try:

        print("Connecting to M1C1 LiDAR...")

        lidar = M1C1Lidar(
            port=LIDAR_PORT,
            baudrate=LIDAR_BAUDRATE,
            angle_offset=192.0,
            min_range=80,
            max_range=8000,
            send_start_cmd=False
        )

        print("Initializing servo...")

        servo = ServoController(
            pin=SERVO_PIN
        )

        scanner = Scanner3D(
            lidar=lidar,
            servo=servo,
            angle_step=DEFAULT_ANGLE_STEP,
            settle_time=SERVO_SETTLE_TIME,
            sweep_timeout=5,
            lidar_offset=20.0
        )

        root = tk.Tk()

        LidarViewer(
            root,
            scanner,
            lidar,
            servo
        )

        root.mainloop()

    except Exception:

        if lidar is not None:
            lidar.close()

        if servo is not None:
            servo.cleanup()

        raise


if __name__ == "__main__":
    main()
