import math
import socket
from gpiozero import AngularServo

UPPER_ARM_CM = 10
FOREARM_CM = 10

SHOULDER_PIN = 17
ELBOW_PIN = 27
GRIPPER_PIN = 22

SHOULDER_OFFSET = 0
ELBOW_OFFSET = 0
GRIPPER_OPEN = 45
GRIPPER_CLOSED = -45

x = 18
z = 8

shoulder = AngularServo(SHOULDER_PIN, min_angle=-90, max_angle=90)
elbow = AngularServo(ELBOW_PIN, min_angle=-90, max_angle=90)
gripper = AngularServo(GRIPPER_PIN, min_angle=-90, max_angle=90)

def move_arm():
    elbow_angle = math.acos(
        (x*x + z*z - UPPER_ARM_CM * UPPER_ARM_CM - FOREARM_CM * FOREARM_CM) 
        / (2*UPPER_ARM_CM * FOREARM_CM)
    )
    shoulder_angle = math.atan2(z,x) - math.atan2(
        FOREARM_CM*math.sin(elbow_angle),
        UPPER_ARM_CM + FOREARM_CM * math.cos(elbow_angle)
    )

    shoulder.angle = math.degrees(shoulder_angle) + SHOULDER_OFFSET
    elbow_angle = math.degrees(elbow_angle) + ELBOW_OFFSET

def forward():
    global x
    x += 0.25
    move_arm()

def backward():
    global x
    x-=0.25
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

def open_gripper():
    gripper.angle = GRIPPER_OPEN

def close_gripper():
    gripper.angle = GRIPPER_CLOSED

host = "192.168.100.25" #PC IP
port = 5005
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((host, port))

home()

while True:
    data,address = server_socket.recvfrom(1024)
    command = data.decode("utf-8").strip()
    print(command)

    if command == "FORWARD":
        forward()
    elif command  == "BACKWARD":
        backward()
    elif command == "RAISE":
        raise_arm()
    elif command == "LOWER":
        lower()
    elif command == "HOME":
        home()
    elif command == "OPEN":
        open_gripper()
    elif command == "CLOSE":
        close_gripper()

#10.42.0.1