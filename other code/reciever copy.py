import math
import socket
from gpiozero import AngularServo

# Change these to match your arm.
FIRST_SEGMENT_CM = 10
SECOND_SEGMENT_CM = 10
THIRD_SEGMENT_CM = 10

SHOULDER_PIN = 17 
ELBOW_PIN = 27
WRIST_PIN = 22
GRIPPER_PITCH_PIN = 23

SHOULDER_OFFSET = 0
ELBOW_OFFSET = 0
WRIST_OFFSET = 0
GRIPPER_PITCH_OFFSET = 0
PITCH_STEP = 5
THIRD_SEGMENT_ANGLE = 90

# Starting gripper position, measured from the shoulder pivot.
x = 0
z = 30
gripper_pitch = 0

shoulder = AngularServo(SHOULDER_PIN, min_angle=-90, max_angle=90)
elbow = AngularServo(ELBOW_PIN, min_angle=-90, max_angle=90)
wrist = AngularServo(WRIST_PIN, min_angle=-90, max_angle=90)
gripper_pitch_servo = AngularServo(GRIPPER_PITCH_PIN, min_angle=-90, max_angle=90)

def move_arm():
    # Desired angle of the third segment
    third_segment_angle = math.radians(THIRD_SEGMENT_ANGLE)

    # Calculate the wrist position (end of segment 2)
    wrist_x = x - THIRD_SEGMENT_CM * math.cos(third_segment_angle)
    wrist_z = z - THIRD_SEGMENT_CM * math.sin(third_segment_angle)

    L1 = FIRST_SEGMENT_CM
    L2 = SECOND_SEGMENT_CM

    # Distance from shoulder to wrist
    distance_sq = wrist_x**2 + wrist_z**2
    distance = math.sqrt(distance_sq)

    # Check whether the target is physically reachable
    if distance > L1 + L2 or distance < abs(L1 - L2):
        print("Target is outside the arm's reachable workspace.")
        return

    # Inverse kinematics
    cos_elbow = (
        distance_sq - L1**2 - L2**2
    ) / (2 * L1 * L2)

    # Avoid numerical errors with acos
    cos_elbow = max(-1.0, min(1.0, cos_elbow))

    # Try both elbow configurations
    elbow_candidates = [
        math.acos(cos_elbow),
        -math.acos(cos_elbow)
    ]

    solution = None

    for elbow_angle in elbow_candidates:
        shoulder_angle = math.atan2(wrist_z, wrist_x) - math.atan2(
            L2 * math.sin(elbow_angle),
            L1 + L2 * math.cos(elbow_angle)
        )

        wrist_angle = (
            third_segment_angle - shoulder_angle - elbow_angle
        )

        shoulder_deg = math.degrees(shoulder_angle) + SHOULDER_OFFSET
        elbow_deg = math.degrees(elbow_angle) + ELBOW_OFFSET
        wrist_deg = math.degrees(wrist_angle) + WRIST_OFFSET

        # Only accept solutions within the configured servo limits
        if (
            -90 <= shoulder_deg <= 90
            and -90 <= elbow_deg <= 90
            and -90 <= wrist_deg <= 90
        ):
            solution = (shoulder_deg, elbow_deg, wrist_deg)
            break

    if solution is None:
        print(
            f"No valid servo solution for x={x:.2f}, z={z:.2f}. "
            "Try another target or adjust the joint calibration."
        )
        return

    shoulder_deg, elbow_deg, wrist_deg = solution

    # Move the joints
    shoulder.angle = shoulder_deg
    elbow.angle = elbow_deg
    wrist.angle = wrist_deg

    # Keep the gripper at the requested pitch
    gripper_pitch_servo.angle = (
        gripper_pitch
        - (shoulder_deg - SHOULDER_OFFSET)
        - (elbow_deg - ELBOW_OFFSET)
        - (wrist_deg - WRIST_OFFSET)
        + GRIPPER_PITCH_OFFSET
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
    x = 0
    z = 30
    move_arm()

def pitch_up():
    global gripper_pitch
    gripper_pitch += PITCH_STEP
    move_arm()

def pitch_down():
    global gripper_pitch
    gripper_pitch -= PITCH_STEP
    move_arm()

host = "192.168.100.210"
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
