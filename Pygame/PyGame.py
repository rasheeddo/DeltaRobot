
import time
import pygame
import math



pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
#print 'Initialized Joystick : %s' % j.get_name()
"""
Returns a vector of the following form:
[LThumbstickX, LThumbstickY, Unknown Coupled Axis???, 
RThumbstickX, RThumbstickY, 
Button 1/X, Button 2/A, Button 3/B, Button 4/Y, 
Left Bumper, Right Bumper, Left Trigger, Right Triller,
Select, Start, Left Thumb Press, Right Thumb Press]
Note:
No D-Pad.
Triggers are switches, not variable. 
Your controller may be different
"""

def map(x):
    result = (x - 1)*(8000 - (-8000)) / (-1 - (1)) + (-8000)
    if result < 4000: return 4000
    if result > 8000: return 8000
    else: return result

def rc_map(x,in_min,in_max,out_min,out_max):
    result = (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min
    return result



def get():
    #Read input from the two joysticks
    pygame.event.pump()
    unknown1 = j.get_axis(0)
    unknown2 = j.get_axis(1)
    throttle = j.get_axis(2)
    roll = j.get_axis(4)
    pitch = j.get_axis(3)
    yaw = j.get_axis(5)
    button1 = j.get_button(0)
    button2 = j.get_button(1)
    rc_angles = [throttle, pitch, roll, yaw, unknown1, unknown2, button1, button2]
    #print("Pitch: {}".format(yaw))
    #print("Roll: {}".format(roll))
    #print("Throttle: {}".format(throttle))
    #print(rc_angles)
    return rc_angles



def test():
    first = time.time()
    while True:
        des_attitude = get()
        des_throttle = rc_map(des_attitude[0],-1,1,-10,10)
        des_pitch = rc_map(des_attitude[1],1,-1,-45,45)
        des_roll = rc_map(des_attitude[2],-1,1,-45,45)
        des_yaw = rc_map(des_attitude[3],-1,1,-45,45)
        des_unknown1 = rc_map(des_attitude[4],-1,1,-45,45)
        des_unknown2 = rc_map(des_attitude[5],-1,1,-45,45)

        print(des_attitude[6])
        print(des_attitude[7])
        #print(des_attitude[2])
        #print(des_attitude[3])
        print("----------------------")


test()
