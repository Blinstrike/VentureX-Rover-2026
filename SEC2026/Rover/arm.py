import gpiozero
import os
os.environ['GPIOZERO_PIN_FACTORY'] = 'pigpio'
positions = []
servos = [
    
]
#gpiozero.AngularServo(17, min_angle=-90, max_angle=90),
#gpiozero.AngularServo(27, min_angle=-90, max_angle=90),
#gpiozero.AngularServo(22, min_angle=-90, max_angle=90),
#gpiozero.AngularServo(23, min_angle=-90, max_angle=90),
#gpiozero.AngularServo(24, min_angle=-90, max_angle=90)

def setpos(pos):
    if pos in positions:
        for servo, angle in zip(servos, positions[pos]):
            servo.value = angle
    else:
        print(f"Position '{pos}' not defined.")

def move(cmd,val):
    try:
        if cmd == "up":
            servos[1].value += val
            servos[2].value += val
        elif cmd == "down":
            servos[1].value -= val
            servos[2].value -= val
        elif cmd == "left":
            servos[0].value -= val
        elif cmd == "right":
            servos[0].value += val
        elif cmd == "fwd":
            servos[3].value += val
            servos[4].value += val
        elif cmd == "rvrs":
            servos[3].value -= val
            servos[4].value -= val
    except Exception as e:
        pass

def release():
    pass
def empty():
    pass