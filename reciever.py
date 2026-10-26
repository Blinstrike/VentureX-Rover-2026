import math
import socket
from gpiozero import AngularServo

# Change these to match your arm. The last two segments stay in a straight line.
FIRST_SEGMENT_CM = 10
SECOND_SEGMENT_CM = 10
THIRD_SEGMENT_CM = 10

SHOULDER_PIN = 17
ELBOW_PIN = 27
GRIPPER_PITCH_PIN = 22

SHOULDER_OFFSET = 0
ELBOW_OFFSET = 0
GRIPPER_PITCH_OFFSET = 0
PITCH_STEP = 5

# This is the starting position of the gripper, measured from the shoulder pivot.
x = 18
z = 8
gripper_pitch = 0

shoulder = AngularServo(SHOULDER_PIN, min_angle=-90, max_angle=90)
elbow = AngularServo(ELBOW_PIN, min_angle=-90, max_angle=90)
gripper_pitch_servo = AngularServo(GRIPPER_PITCH_PIN, min_angle=-90, max_angle=90)

def move_arm():
    end_length = SECOND_SEGMENT_CM + THIRD_SEGMENT_CM
    elbow_angle = math.acos(
        (x * x + z * z - FIRST_SEGMENT_CM * FIRST_SEGMENT_CM - end_length * end_length)
        / (2 * FIRST_SEGMENT_CM * end_length)
    )
    shoulder_angle = math.atan2(z, x) - math.atan2(
        end_length * math.sin(elbow_angle),
        FIRST_SEGMENT_CM + end_length * math.cos(elbow_angle)
    )

    shoulder.angle = math.degrees(shoulder_angle) + SHOULDER_OFFSET
    elbow.angle = math.degrees(elbow_angle) + ELBOW_OFFSET
    gripper_pitch_servo.angle = (
        gripper_pitch - math.degrees(shoulder_angle + elbow_angle) + GRIPPER_PITCH_OFFSET
    )

def forward():
    global x
    x += 0.25
    move_arm()

def backward():
    global x
    x -= 0.25
    move_arm()

def raise_arm():
    global z
    z += 0.25
    move_arm()

def lower():
    global z
    z -= 0.25
    move_arm()

def home():
    global x, z
    x = 18
    z = 8
    move_arm()

def pitch_up():
    global gripper_pitch
    gripper_pitch += PITCH_STEP
    move_arm()

def pitch_down():
    global gripper_pitch
    gripper_pitch -= PITCH_STEP
    move_arm()

host = "192.168.1.136"
port = 5005
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((host, port))

home()

while True:
    data, address = server_socket.recvfrom(1024)
    command = data.decode("utf-8").strip()
    print(command)

    if command == "FORWARD":
        forward()
    elif command == "BACKWARD":
        backward()
    elif command == "RAISE":
        raise_arm()
    elif command == "LOWER":
        lower()
    elif command == "HOME":
        home()
    elif command == "PITCH_UP":
        pitch_up()
    elif command == "PITCH_DOWN":
        pitch_down()
