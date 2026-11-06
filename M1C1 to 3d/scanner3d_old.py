
import math


# ============================================================
# CONFIGURATION
# ============================================================

# Distance between the servo rotation axis and LiDAR origin.
LIDAR_OFFSET_MM = 20.0


# ============================================================
# 3D COORDINATE CONVERSION
# ============================================================

def sweep_to_3d(sweep, phi, offset=LIDAR_OFFSET_MM):
    """
    Convert a 2D LiDAR sweep into 3D coordinates.

    Args:
        sweep: List of (theta, distance) measurements.
        phi: Servo angle in degrees.
        offset: Distance between servo shaft and LiDAR origin (mm).

    Returns:
        List of (x, y, z) coordinates in mm.

    Assumptions:
        - Servo rotation axis is parallel to X.
        - LiDAR is 20 mm above the servo shaft at 0 degrees.
        - LiDAR rotates together with the servo.
    """

    phi = math.radians(phi)

    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)

    points = []

    # Position of LiDAR origin relative to servo shaft
    offset_y = -offset * sin_phi
    offset_z = offset * cos_phi

    for theta, r in sweep:

        theta = math.radians(theta)

        # Convert LiDAR polar coordinates into 3D
        x = r * math.sin(theta)

        y = r * math.cos(theta) * cos_phi

        z = r * math.cos(theta) * sin_phi

        # Apply LiDAR mounting offset
        y += offset_y
        z += offset_z

        points.append((x, y, z))

    return points


# ============================================================
# 3D SCANNER
# ============================================================

class Scanner3D:

    def __init__(
        self,
        lidar,
        servo,
        angle_step=2.0,
        settle_time=0.3,
        sweep_timeout=5,
        lidar_offset=LIDAR_OFFSET_MM
    ):

        self.lidar = lidar
        self.servo = servo

        self.angle_step = float(angle_step)
        self.settle_time = settle_time
        self.sweep_timeout = sweep_timeout

        self.lidar_offset = float(lidar_offset)

    # ========================================================
    # CONFIGURATION
    # ========================================================

    def set_angle_step(self, step):

        step = float(step)

        if not 0 < step <= 180:
            raise ValueError("Invalid angle step")

        self.angle_step = step

    def set_lidar_offset(self, offset):

        self.lidar_offset = float(offset)

    # ========================================================
    # SERVO ANGLES
    # ========================================================

    def get_servo_angles(self):

        angles = []

        angle = -90.0

        while angle <= 90.0:

            angles.append(angle)

            angle += self.angle_step

        if angles[-1] < 90.0:
            angles.append(90.0)

        return angles

    # ========================================================
    # SCANNING
    # ========================================================

    def scan_position(self, angle):

        # Move servo to requested angle
        self.servo.set_angle(
            angle,
            settle_time=self.settle_time
        )

        # Acquire one complete 360-degree LiDAR sweep
        sweep = self.lidar.get_data(
            timeout=self.sweep_timeout
        )

        # Convert measurements to 3D and compensate
        # for the physical distance between the LiDAR
        # measurement origin and servo rotation axis.
        points = sweep_to_3d(
            sweep,
            angle,
            offset=self.lidar_offset
        )

        return points

    # ========================================================
    # RESET
    # ========================================================

    def reset(self):

        self.servo.set_angle(
            -90,
            settle_time=1.0
        )

    # ========================================================
    # CLEANUP
    # ========================================================

    def cleanup(self):

        self.reset()
