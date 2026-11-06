import math

LIDAR_OFFSET_MM = 20.0

def sweep_to_3d(sweep, phi, offset=LIDAR_OFFSET_MM, angle_sign=1.0,
                angle_zero=0.0, servo_sign=1.0):
    """
    Convert one M1C1 2-D sweep at a servo tilt into XYZ millimetres.

    Coordinate convention (matches the original scanner comments):
      * servo shaft is the X axis
      * at servo=0, LiDAR scans the XY plane
      * positive servo angle rotates the scan plane toward +Z
      * LiDAR origin is offset +Z from the shaft at servo=0

    angle_zero / angle_sign are calibration controls, not guessed corrections.
    """
    phi = math.radians(float(phi) * float(servo_sign))
    cp, sp = math.cos(phi), math.sin(phi)
    points = []
    oy = -float(offset) * sp
    oz = float(offset) * cp

    for theta, distance in sweep:
        r = float(distance)
        if not math.isfinite(r) or r <= 0:
            continue
        theta = math.radians((float(theta) - float(angle_zero)) * float(angle_sign))
        x = r * math.sin(theta)
        radial = r * math.cos(theta)
        y = radial * cp + oy
        z = radial * sp + oz
        points.append((x, y, z))
    return points


class Scanner3D:
    def __init__(self, lidar, servo, angle_step=2.0, settle_time=0.15,
                 sweep_timeout=5, lidar_offset=LIDAR_OFFSET_MM):
        self.lidar = lidar
        self.servo = servo
        self.angle_step = float(angle_step)
        self.settle_time = float(settle_time)
        self.sweep_timeout = float(sweep_timeout)
        self.lidar_offset = float(lidar_offset)

    def set_angle_step(self, step):
        step = float(step)
        if not math.isfinite(step) or not 0 < step <= 180:
            raise ValueError("Angle step must be greater than 0 and at most 180 degrees")
        self.angle_step = step

    def get_servo_angles(self):
        # Include both endpoints exactly; avoid floating-point accumulation.
        count = int(math.floor(180.0 / self.angle_step))
        angles = [-90.0 + i * self.angle_step for i in range(count + 1)]
        if not math.isclose(angles[-1], 90.0, abs_tol=1e-9):
            angles.append(90.0)
        else:
            angles[-1] = 90.0
        return angles

    def scan_position(self, angle):
        self.servo.set_angle(angle, settle_time=self.settle_time)
        sweep = self.lidar.get_data(timeout=self.sweep_timeout)
        return sweep_to_3d(sweep, angle, offset=self.lidar_offset)

    def reset(self):
        self.servo.set_angle(-90, settle_time=1.0)

    def cleanup(self):
        self.reset()
