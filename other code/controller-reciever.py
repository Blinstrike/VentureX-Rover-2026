import socket as socket
import gpiozero as GPIO
import time
hostname = socket.gethostname()
host = "192.168.1.137"
port = 5005
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((host, port))
motors = [GPIO.Motor(forward=2, backward=3),
          GPIO.Motor(forward=4, backward=17),
          GPIO.Motor(forward=27, backward=22),
          GPIO.Motor(forward=10, backward=9),
          GPIO.Motor(forward=11, backward=0),
          GPIO.Motor(forward=5, backward=6)]
while True:
    data,add = server_socket.recvfrom(1024)
    cmd = data.decode("utf-8").strip()
    cmd,val = cmd.split("`")
    print(cmd)
    if cmd == "FORWARD":
        for motor in motors:
            motor.forward()
        time.sleep(3)
        for motor in motors:
            motor.stop()
    elif cmd == "LEFT":
        motors[1].forward()
        motors[3].forward()
        motors[5].forward()
        time.sleep(3)
        for motor in motors:
            motor.stop()
    elif cmd == "ROTATEL":
        motors[0].backward()
        motors[2].backward()
        motors[4].backward()
        motors[1].forward()
        motors[3].forward()
        motors[5].forward()
        time.sleep(3)
        for motor in motors:
            motor.stop()
    elif cmd == "RIGHT":
        motors[0].forward()
        motors[2].forward()
        motors[4].forward()
        time.sleep(3)
        for motor in motors:
            motor.stop()
    elif cmd == "REVERSE":
        for motor in motors:
            motor.backward()
        time.sleep(3)
        for motor in motors:
            motor.stop()
    elif cmd == "ROTATER":
        motors[0].forward()
        motors[2].forward()
        motors[4].forward()
        motors[1].backward()
        motors[3].backward()
        motors[5].backward()
        time.sleep(3)
        for motor in motors:
            motor.stop()