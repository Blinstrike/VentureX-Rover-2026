
from gpiozero import AngularServo
from time import sleep
import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'

class ServoController:

    def __init__(self, pin=17):

        self.servo = AngularServo(
            pin,
            min_angle=0,
            max_angle=180,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025,
            initial_angle=None
        )

        self.angle = None

    def set_angle(self, angle, settle_time=0.3):
        """
        Logical angle: -90 to +90 degrees.
        Physical servo angle: 0 to 180 degrees.
        """

        angle = max(-90, min(90, float(angle)))

        physical_angle = angle + 90

        self.servo.angle = physical_angle
        self.angle = angle

        print(
            f"Servo: {angle:+.1f}° "
            f"(physical: {physical_angle:.1f}°)"
        )

        sleep(settle_time)

    def cleanup(self):
        self.servo.detach()
        self.servo.close()
