import json
import socket
import time
import drive
import arm
from communication import *
import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'
from lidar_service import LidarService

# LiDAR stream uses TCP 5006; existing rover control remains on TCP 5005.
lidar_service = LidarService(port=5006)
lidar_service.start()

IP = ""

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((IP, 5005))
server_socket.listen(1)

print("Init done, awaiting base station connection...")

while True:
    try:
        client_socket,client_address = server_socket.accept()
        print(f'Connected to: {client_address}')
        while True:
            packet = recv_msg(client_socket)
            if packet is None:
                raise socket.error("Base station disconnected.")
            print(packet)

            if packet['type'] == 'scan_lidar':
                lidar_service.start_scan()
            elif packet['type'] == 'stop_lidar':
                lidar_service.stop_scan()
            elif packet['type']=='drive':
                if(packet['cmd']=="move"):
                    drive.move(packet['val'])
                elif(packet['cmd']=="turn"):
                    drive.turn(packet['val'])
                elif(packet['cmd']=="rotate"):
                    drive.rotate(packet['val'])
                elif(packet['cmd']=="stop"):
                    drive.stop()
            elif packet['type']=='arm':
                drive.stop()
                if(packet['cmd'] == "pos"):
                    arm.setpos(packet['val'])
                elif(packet['cmd'] == "release"):
                    arm.release()
                elif(packet['cmd'] == "empty"):
                    arm.empty()
                else:
                    arm.move(packet['cmd'], packet['val'])
    except (socket.error, ConnectionResetError, BrokenPipeError) as e:
        drive.stop()
        print(f'\n[EROARE DE CONEXIUNE]: {e}')


