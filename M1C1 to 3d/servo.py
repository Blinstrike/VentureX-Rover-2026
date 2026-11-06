import math
from time import sleep

from gpiozero import AngularServo


# The servo has a nominal 270-degree physical range, assumed to correspond
# to 0.5–2.5 ms. Use only its central 180 degrees: -90 to +90.
#
# For a 270-degree servo, the central +/-90-degree section is:
#   0.5 ms + (45/270)*2.0 ms = 0.8333 ms
#   0.5 ms + (225/270)*2.0 ms = 2.1667 ms
#
# Confirm these pulse widths against your exact servo datasheet and calibrate
# cautiously before allowing it to reach either endpoint.
SERVO_MIN_ANGLE = -90.0
SERVO_MAX_ANGLE = 90.0
SERVO_MIN_PULSE_WIDTH = 0.5 / 1000 + (45.0 / 270.0) * (2.0 / 1000)
SERVO_MAX_PULSE_WIDTH = 0.5 / 1000 + (225.0 / 270.0) * (2.0 / 1000)


class ServoController:
    def __init__(self, pin=17):
        self.servo = AngularServo(
            pin,
            min_angle=SERVO_MIN_ANGLE,
            max_angle=SERVO_MAX_ANGLE,
            min_pulse_width=SERVO_MIN_PULSE_WIDTH,
            max_pulse_width=SERVO_MAX_PULSE_WIDTH,
            initial_angle=None,
        )
        self.angle = None

    def set_angle(self, angle, settle_time=0.3):
        """Move to a physical servo angle between -90 and +90 degrees."""
        angle = float(angle)
        if not math.isfinite(angle):
            raise ValueError("Servo angle must be a finite number")

        angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))
        self.servo.angle = angle
        self.angle = angle

        print(f"Servo physical angle: {angle:+.1f}°")
        sleep(max(0.0, float(settle_time)))

    def cleanup(self):
        self.servo.detach()
        self.servo.close()
