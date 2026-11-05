import gpiozero as GPIO
import time 

motors = [GPIO.Motor(forward=22, backward=23, pwm=True),
          GPIO.Motor(forward=17, backward=27, pwm=True)]

def move(val):
    if(val<0):
        for i in motors:
            i.forward(-1*val)
    if(val>0):
        for i in motors:
            i.backward(val)

def turn(val):
    if(val>0):
        for i in range(0, len(motors), 2):
            motors[i].forward(abs(val))
            motors[i+1].forward(abs(val)/2)
    if(val<0):
        for i in range(0, len(motors), 2):
            motors[i].forward(abs(val)/4)
            motors[i+1].forward(abs(val)/2)

def rotate(val):
    if(val>0):
        for i in range(0, len(motors), 2):
            motors[i].forward(abs(val))
            motors[i+1].backward(abs(val))
    if(val<0):
        for i in range(0, len(motors), 2):
            motors[i].backward(abs(val))
            motors[i+1].forward(abs(val))

def stop():
    for i in motors:
        i.stop()
            
