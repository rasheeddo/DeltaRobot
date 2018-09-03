# Test servo with Maestro 

import maestro
import time

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

servo = maestro.Controller()
pos0 = servo.getPosition(0)
pre_pos = map(pos0,4000, 8000, 0, 90)

print("pos0=%f" %pos0)
print("pre_pos=%f" %pre_pos)

pos0 = servo.getPosition(0)

acc = 4
servo.setAccel(1,acc)      #set servo 0 acceleration to 4
servo.setAccel(1,acc)      #set servo 0 acceleration to 4
servo.setAccel(2,acc)      #set servo 0 acceleration to 4
'''
speed = 60
servo.setSpeed(0,speed)
servo.setSpeed(1,speed)
servo.setSpeed(2,speed)
'''
pos = 0
servo_pos = map(pos, -70, 20, 4000, 8000)
t1 = time.time()
servo.setTarget(0,servo_pos-400)  #set servo to move to center position  min=3000  mid=6000  max=9000
servo.setTarget(1,servo_pos-600)
servo.setTarget(2,servo_pos-400)

while servo.isMoving(0):
	print("servo still running...")

t2 = time.time()
period = t2 - t1
print("time=%f" %period)
print("input_pos=%f" %pos)
print("servo_pos=%f" %servo_pos)
print("-----------------------------")


servo.close