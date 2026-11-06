#sudo systemctl enable pigpiod
#sudo systemctl start pigpiod
import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'
import gpiozero as gpio
import matplotlib.pyplot as plt
import numpy as np
import time


servo = gpio.AngularServo(17, min_angle=-90, max_angle=90)  # Initialize the servo on GPIO pin 17
servo.angle = 0  # Set the servo to the middle position
servo_angle = 0  # Initialize the servo position variable
# Move the servo back to the middle position
time.sleep(1)
while True:
    # Move the servo to the left
    while servo_angle > -90:
        servo_angle -= 1  # Decrease the servo angle
        if servo_angle < -90:  # Limit the servo angle to -90
            servo_angle = -90
        servo.angle = servo_angle  # Set the servo angle
        time.sleep(0.01)  # Wait for a short time

    # Move the servo to the right
    while servo_angle < 90:
        servo_angle += 1  # Increase the servo angle
        if servo_angle > 90:  # Limit the servo angle to 90
            servo_angle = 90
        servo.angle = servo_angle  # Set the servo angle
        time.sleep(0.01)  # Wait for a short time
