import socket as socket
import gpiozero as GPIO
import time
hostname = socket.gethostname()
host = "0.0.0.0"
port = 5005
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((host, port))

shoulder = GPIO.AngularServo(15, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
elbow = GPIO.AngularServo(18, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
hand = GPIO.AngularServo(23, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
effector = GPIO.AngularServo(24, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
gripper = GPIO.AngularServo(25, min_angle=-90, max_angle=90, min_pulse_width=0.0005, max_pulse_width=0.0025)
while True:
    data,add = server_socket.recvfrom(1024)
    cmd = data.decode("utf-8").strip()
    print(cmd)
    if cmd == "HOME":
        shoulder.angle = 0
        elbow.angle = 0
        hand.angle = 0
        effector.angle = 0
    else:
        shoulder_angle, elbow_angle, hand_angle, effector_angle, gripper_angle = map(int, cmd.split(","))
        shoulder.angle = shoulder_angle
        elbow.angle = elbow_angle
        hand.angle = hand_angle
        effector.angle = effector_angle
    