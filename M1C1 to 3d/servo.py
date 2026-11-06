import math
import os
from time import sleep

# Select the GPIO backend before importing gpiozero.
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "pigpio")

from gpiozero import AngularServo


# LiDAR coordinates remain -90°..+90°.
# They map onto the 270° servo's physical range, 0°..270°.
LOGICAL_MIN_ANGLE = -90.0
LOGICAL_MAX_ANGLE = 90.0
SERVO_MIN_ANGLE = 0.0
SERVO_MAX_ANGLE = 270.0


class ServoController:
    def __init__(self, pin=17):
        self.servo = AngularServo(
            pin,
            min_angle=SERVO_MIN_ANGLE,
            max_angle=SERVO_MAX_ANGLE,
            # Verify these pulse widths against the servo datasheet.
            min_pulse_width=0.0005,
            max_pulse_width=0.0025,
            initial_angle=None,
        )
        self.angle = None

    @staticmethod
    def _to_physical_angle(logical_angle):
        """Map logical -90°..+90° to physical servo 0°..270°."""
        logical_angle = max(
            LOGICAL_MIN_ANGLE,
            min(LOGICAL_MAX_ANGLE, float(logical_angle)),
        )
        fraction = (
            (logical_angle - LOGICAL_MIN_ANGLE)
            / (LOGICAL_MAX_ANGLE - LOGICAL_MIN_ANGLE)
        )
        return SERVO_MIN_ANGLE + fraction * (
            SERVO_MAX_ANGLE - SERVO_MIN_ANGLE
        )

    def set_angle(self, angle, settle_time=0.3):
        """Set the LiDAR's logical tilt angle, from -90° through +90°."""
        angle = float(angle)
        if not math.isfinite(angle):
            raise ValueError("Servo angle must be a finite number")

        angle = max(LOGICAL_MIN_ANGLE, min(LOGICAL_MAX_ANGLE, angle))
        physical_angle = self._to_physical_angle(angle)

        self.servo.angle = physical_angle
        self.angle = angle

        print(
            f"LiDAR angle: {angle:+.1f}° "
            f"(270° servo position: {physical_angle:.1f}°)"
        )
        sleep(max(0.0, float(settle_time)))

    def cleanup(self):
        self.servo.detach()
        self.servo.close()
