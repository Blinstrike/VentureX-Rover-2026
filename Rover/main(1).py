import sys
import time
import socket
import struct
import json
import threading
import subprocess
import os
import pygame
import communication as comm # Conține send_msg și recv_msg bazate pe JSON și prefix de lungime
import utils

pygame.init()
pygame.joystick.init()

# Verificare controller
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No controller found. Please connect your Marvo GT-84 and try again.")
    sys.exit()

controller = pygame.joystick.Joystick(0)
controller.init()
print(f"Successfully connected to: {controller.get_name()}")

ROVER_IP = "10.42.0.1"  # Pune IP-ul real al Raspberry Pi-ului tău
ROVER_PORT = 5005

def connect_to_rover():
    while True:
        try:
            print(f"Se încearcă conectarea la rover ({ROVER_IP})...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0) 
            sock.connect((ROVER_IP, ROVER_PORT))
            sock.settimeout(None) 
            print("Conectat cu succes la Rover!")
            return sock
        except (socket.error, socket.timeout):
            print("Roverul nu răspunde. Reîncercare în 1 secundă...")
            time.sleep(1)

# Pornirea conexiunii inițiale
sock = connect_to_rover()

# Container mutabil pentru thread-ul de heartbeat
sock_container = [sock]
lidar_viewer_process = None
lidar_scanning = False

def heartbeat_thread_func(container):
    """ Trimite un ping automat la fiecare secundă dacă conexiunea este activă """
    while True:
        time.sleep(1.0)
        current_sock = container[0]
        if current_sock:
            try:
                ping_packet = {"type": "ping"}
                comm.send_msg(current_sock, ping_packet)
            except (socket.error, BrokenPipeError):
                pass 

# Pornim thread-ul de fundal pentru Heartbeat (Ping)
heartbeat_thread = threading.Thread(target=heartbeat_thread_func, args=(sock_container,), daemon=True)
heartbeat_thread.start()

control_mode = "drive" 

#Variables for drive mode
last_drive_val_1 = 0.0
last_drive_val_0 = 0.0
last_drive_val_2 = 0.0

#Variables for arm mode
last_arm_val_1 = 0.0
last_arm_val_0 = 0.0
last_arm_val_2 = 0.0

#Keys for controlling mech arm
KEY_ARM_MAP = {
    pygame.K_e: "up",
    pygame.K_q: "down",
    pygame.K_d: "right",
    pygame.K_a: "left",
    pygame.K_w: "forward",
    pygame.K_s: "backward"
}



running = True
clock = pygame.time.Clock()

print("\n--- Sistemul este pregătit. Folosește controllerul pentru comenzi ---")

while running:
    try:
        # Păstrăm variabile locale pentru comanda curentă din acest cadru (frame)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} PRESSED")
                # Button 0 toggles LiDAR scanning. Driving stays disabled
                # until scanning is explicitly stopped.
                if event.button == 0:
                    lidar_scanning = not lidar_scanning
                    if lidar_scanning:
                        print("[PC] LiDAR scan started; driving disabled.")
                        comm.send_msg(sock_container[0], {"type": "drive", "cmd": "stop"})
                        comm.send_msg(sock_container[0], {"type": "scan_lidar"})
                        last_drive_val_1 = last_drive_val_0 = last_drive_val_2 = 0.0
                        # Launch the remote viewer once (TCP port 5006).
                        if lidar_viewer_process is None or lidar_viewer_process.poll() is not None:
                            viewer_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lidar_viewer.py")
                            lidar_viewer_process = subprocess.Popen(
                                [sys.executable, viewer_path, "--remote", ROVER_IP]
                            )
                    else:
                        print("[PC] LiDAR scan stopped; driving re-enabled.")
                        comm.send_msg(sock_container[0], {"type": "stop_lidar"})
                        comm.send_msg(sock_container[0], {"type": "drive", "cmd": "stop"})
                        last_drive_val_1 = last_drive_val_0 = last_drive_val_2 = 0.0
                if event.button == 5:
                    if control_mode == "drive": control_mode = "arm"
                    else: control_mode = "drive"
                    print(f"[PC] Mod control schimbat la: {control_mode}")
            elif event.type == pygame.JOYBUTTONUP:
                print(f"Button {event.button} RELEASED")
            elif event.type == pygame.JOYAXISMOTION:
                if control_mode == "drive" and not lidar_scanning:
                    # Vertical (Axa 1) - Mers înainte / înapoi
                    val_1 = utils.map_axis(controller.get_axis(1))
                    if val_1 != last_drive_val_1:
                        packet = { "type": "drive", "cmd": "move", "val": val_1 }
                        comm.send_msg(sock_container[0], packet)
                        last_drive_val_1 = val_1

                    # Horizontal (Axa 0) - Viraj stânga / dreapta
                    val_0 = utils.map_axis(controller.get_axis(0))
                    if val_0 != last_drive_val_0:
                        packet = { "type": "drive", "cmd": "turn", "val": val_0 }
                        comm.send_msg(sock_container[0], packet)
                        last_drive_val_0 = val_0

                    # Horizontal (Axa 2) - Rotire pe loc
                    val_2 = utils.map_axis(controller.get_axis(2))
                    if val_2 != last_drive_val_2:
                        packet = { "type": "drive", "cmd": "rotate", "val": val_2 }
                        comm.send_msg(sock_container[0], packet)
                        last_drive_val_2 = val_2
            elif event.type == pygame.JOYHATMOTION:
                print(f"D-Pad moved to {event.value}")
            elif control_mode == "arm":
                if event.type == pygame.KEYDOWN:
                    if event.key in KEY_ARM_MAP:
                        cmd = KEY_ARM_MAP[event.key]
                        packet = {"type": "arm", "cmd": cmd, "val": 1.0}
                        comm.send_msg(sock_container[0], packet)
                # La eliberare (KEYUP)
                    elif event.type == pygame.KEYUP:
                        if event.key in KEY_ARM_MAP:
                            cmd = KEY_ARM_MAP[event.key]
                            packet = {"type": "arm", "cmd": cmd, "val": 0.0}  # Trimite 0.0 pe aceeași comandă pentru oprire
                            comm.send_msg(sock_container[0], packet)
        clock.tick(60)

    except (socket.error, BrokenPipeError, ConnectionResetError):
        print("\n[AVERTISMENT] Trimiterea a eșuat. Wi-Fi-ul a picat în peșteră!")
        new_sock = connect_to_rover()
        sock_container[0] = new_sock

pygame.quit()
