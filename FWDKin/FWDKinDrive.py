# Test servo with Maestro 

import time
import numpy
import math
import pygame
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

######################################## Pygame #############################################
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

def rc_map(x,in_min,in_max,out_min,out_max):
    result = (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min
    return result

def getButton():
    #Read input from the two joysticks
    pygame.event.pump()
    #unknown1 = j.get_axis(0)
    #unknown2 = j.get_axis(1)
    #throttle = j.get_axis(2)
    #roll = j.get_axis(4)
    #pitch = j.get_axis(3)
    #yaw = j.get_axis(5)
    button0 = j.get_button(0)
    button1 = j.get_button(1)
    button2 = j.get_button(2)
    button3 = j.get_button(3)
    button4 = j.get_button(4)
    button5 = j.get_button(5)
    button6 = j.get_button(6)
    button7 = j.get_button(7)
    button8 = j.get_button(8)
    button9 = j.get_button(9)
    button10 = j.get_button(10)
    joy_button = [button0, button1, button2, button3, button4, button5, button6, button7,button8, button9, button10]
    
    return joy_button

def getAxis():
    #Read input from the two joysticks
    pygame.event.pump()
    axis0 = j.get_axis(0)
    axis1 = j.get_axis(1)
    axis2 = j.get_axis(2)
    axis3 = j.get_axis(4)
    axis4 = j.get_axis(3)
    axis5 = j.get_axis(5)
    joy_axis = [axis0, axis1, axis2, axis3, axis4, axis5]
    return joy_axis

def getHat():
    pygame.event.pump()
    hat0 = j.get_hat(0)
    
    joy_hat = hat0
    return joy_hat


def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def SetOperatingMode(MODE):
    
    TorqueOff()

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, MODE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, MODE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, MODE)


    present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE)
    if present_mode == 0:
        # Current (Torque) Control Mode
        print("Now Operating Mode is Torque Control")
    elif present_mode == 3:
        # Position Control Mode
        print("Now Operating Mode is Position Control")
    elif present_mode == 5:
        # Current-based Position Control Mode
        print("Now Operating Mode is Current-based Position Control")
    else:
        print("In other Mode that didn't set!")

def SetGoalCurrent(SetCur):

    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_CURRENT, SetCur)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Goal Current is set")

def DeltaGoHome():

	SetProfile1(30,20)
	SetProfile2(30,20)
	SetProfile3(30,20)

	pos1 = 90
	pos2 = 90
	pos3 = 90

	servo_ang1 = map(pos1, 0.0, 360.0, 0, 4095)
	servo_ang2 = map(pos2, 0.0, 360.0, 0, 4095)
	servo_ang3 = map(pos3, 0.0, 360.0, 0, 4095)

	dxl1_goal_position = servo_ang1
	dxl2_goal_position = servo_ang2
	dxl3_goal_position = servo_ang3

	dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
	dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
	dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))

	print("Delta Robot is backing home...")
	Moving1, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MOVING)
	Moving2, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MOVING)
	Moving3, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_MOVING)

	while((Moving1==1) and (Moving2==1) and (Moving3==1)):
		Moving1, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MOVING)
		Moving2, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MOVING)
		Moving3, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_MOVING)

	print("Delta Robot is in home postion")

def GripperCheck():
    servo_com4 = 1500
    dxl4_goal_position = int(servo_com4)
    dxl_comm_result4, dxl_error4 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
    time.sleep(1)
    servo_com4 = 3600
    dxl4_goal_position = int(servo_com4)
    dxl_comm_result4, dxl_error4 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
    time.sleep(1)
    servo_com4 = 1500
    dxl4_goal_position = int(servo_com4)
    dxl_comm_result4, dxl_error4 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_goal_position)

def DeltaFWD(deg1, deg2, deg3):

    rad2deg = 180/math.pi
    deg2rad = math.pi/180
    ########################## parameters ################################
    sb = 315.339
    sp = 88.335
    L = 293         #270
    l = 555         #545
    h = 70
    root3 = math.sqrt(3)
    wb = (root3/6)*sb
    ub = (root3/3)*sb
    wp = (root3/6)*sp
    up = (root3/3)*sp

    deg1 = deg1*deg2rad
    deg2 = deg2*deg2rad
    deg3 = deg3*deg2rad

    A1v = numpy.array([0, -wb - L*math.cos(deg1) + up, -L*math.sin(deg1)])
    A2v = numpy.array([(root3/2)*(wb + L*math.cos(deg2)) - sp/2, 0.5*(wb+ L*math.cos(deg2)) - wp, -L*math.sin(deg2)])
    A3v = numpy.array([(-root3/2)*(wb + L*math.cos(deg3)) + sp/2, 0.5*(wb+ L*math.cos(deg3)) - wp, -L*math.sin(deg3)])

    r1 = l
    r2 = l
    r3 = l

    x1 = A1v[0]
    y1 = A1v[1]
    z1 = A1v[2]

    x2 = A2v[0]
    y2 = A2v[1]
    z2 = A2v[2]

    x3 = A3v[0]
    y3 = A3v[1]
    z3 = A3v[2]

    #Select the method to calculate
    #Depends on the height of virtual spheres center
    #Method 1 is used when the height of z1 z2 z3 are equal
    #Method 2, 3, 4 are trying to avoid 0 division at a13 and a23

    if ((z1==z2) and (z2==z3) and (z1==z3)):
        method = 1
    elif ((z1 != z3) and (z2 != z3)):
        method = 2
    elif ((z1 != z2) and (z1 != z3)):
        method = 3
    else:
        method = 4

    if method == 1:
        zn = z1  # z1 = z2 = z3 = zn

        a = 2*(x3 - x1)
        b = 2*(y3 - y1)
        c = r1**2 - r3**2 - x1**2 - y1**2 + x3**2 + y3**2
        d = 2*(x3 - x2)
        e = 2*(y3 - y2)
        f = r2**2 - r3**2 - x2**2 -y2**2 + x3**2 + y3**2

        numX = c*e - b*f
        denX = a*e - b*d
        x = numX/denX
        if x < 0.000001:
            x = 0

        numY = a*f - c*d
        denY = a*e - b*d
        y = numY/denY
        if y < 0.000001:
            y = 0

        A = 1
        B = -2*zn
        C = zn**2 - r1**2 + (x-x1)**2 + (y-y1)**2

        z = [None]*2

        z[0] = (-B + math.sqrt(B**2 - 4*C))/2;
        z[1] = (-B - math.sqrt(B**2 - 4*C))/2;

        realANS = [None]*3

        if z[0] < 0: 
            realANS = numpy.array([x,y,z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x,y,z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)

        print("Method: %d" %method)
        print("FWD_X:%f" %realANS[0])
        print("FWD_Y:%f" %realANS[1])
        print("FWD_Z:%f" %realANS[2])

    elif method ==2:

        a11 = 2*(x3 - x1)
        a12 = 2*(y3 - y1)
        a13 = 2*(z3 - z1)
        
        a21 = 2*(x3 - x2)
        a22 = 2*(y3 - y2)
        a23 = 2*(z3 - z2)
        
        b1 = r1**2 - r3**2 - x1**2 - y1**2 - z1**2 + x3**2 + y3**2 + z3**2
        b2 = r2**2 - r3**2 - x2**2 - y2**2 - z2**2 + x3**2 + y3**2 + z3**2
        
        a1 = (a11/a13) - (a21/a23)
        a2 = (a12/a13) - (a22/a23)
        a3 = (b2/a23) - (b1/a13)

        a4 = -a2/a1
        a5 = -a3/a1
        a6 = (-a21*a4 - a22)/a23
        a7 = (b2 - a21*a5)/a23
        
        a = a4**2 + 1 + a6**2;
        b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1);
        c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2;
        '''
        YY = Symbol('YY')
        sol = solve(a*YY**2 + b*YY + c,YY)
        y = sol
        '''
        y = [None]*2
        y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
        y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

        x = [None]*2
        z = [None]*2

        x[0] = a4*y[0] + a5;
        x[1] = a4*y[1] + a5;
        z[0] = a6*y[0] + a7;
        z[1] = a6*y[1] + a7;

        realANS = [None]*3
        
        if z[0] < 0:
            realANS = numpy.array([x[0],y[0],z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x[1],y[1],z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)

        print("Method: %d" %method)
        print("FWD_X:%f" %realANS[0])
        print("FWD_Y:%f" %realANS[1])
        print("FWD_Z:%f" %realANS[2])

    elif method == 3:
        a11 = 2*(x1 - x2)
        a12 = 2*(y1 - y2)
        a13 = 2*(z1 - z2)
        
        a21 = 2*(x1 - x3)
        a22 = 2*(y1 - y3)
        a23 = 2*(z1 - z3)
        
        b1 = r2**2 - r1**2 - x2**2 - y2**2 - z2**2 + x1**2 + y1**2 + z1**2
        b2 = r3**2 - r1**2 - x3**2 - y3**2 - z3**2 + x1**2 + y1**2 + z1**2
        
        a1 = (a11/a13) - (a21/a23)
        a2 = (a12/a13) - (a22/a23)
        a3 = (b2/a23) - (b1/a13)

        a4 = -a2/a1
        a5 = -a3/a1
        a6 = (-a21*a4 - a22)/a23
        a7 = (b2 - a21*a5)/a23
        
        a = a4**2 + 1 + a6**2
        b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1)
        c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2
        '''
        YY = Symbol('YY')
        sol = solve(a*YY**2 + b*YY + c,YY);
        y = sol
        '''
        y = [None]*2
        y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
        y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

        x = [None]*2
        z = [None]*2

        x[0] = a4*y[0] + a5
        x[1] = a4*y[1] + a5
        z[0] = a6*y[0] + a7
        z[1] = a6*y[1] + a7

        realANS = [None]*3

        if z[0] < 0: 
            realANS = numpy.array([x[0],y[0],z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x[1],y[1],z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)

        print("Method: %d" %method)
        print("FWD_X:%f" %realANS[0])
        print("FWD_Y:%f" %realANS[1])
        print("FWD_Z:%f" %realANS[2])

    if method == 4:
        a11 = 2*(x2 - x1)
        a12 = 2*(y2 - y1)
        a13 = 2*(z2 - z1)
        
        a21 = 2*(x2 - x3)
        a22 = 2*(y2 - y3)
        a23 = 2*(z2 - z3)
        
        b1 = r1**2 - r2**2 - x1**2 - y1**2 - z1**2 + x2**2 + y2**2 + z2**2
        b2 = r3**2 - r2**2 - x3**2 - y3**2 - z3**2 + x2**2 + y2**2 + z2**2
        
        a1 = (a11/a13) - (a21/a23)
        a2 = (a12/a13) - (a22/a23)
        a3 = (b2/a23) - (b1/a13)

        a4 = -a2/a1
        a5 = -a3/a1
        a6 = (-a21*a4 - a22)/a23
        a7 = (b2 - a21*a5)/a23
        
        a = a4**2 + 1 + a6**2
        b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1)
        c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2
        '''
        YY = Symbol('YY')
        sol = solve(a*YY**2 + b*YY + c,YY);
        y = sol
        '''
        y = [None]*2
        y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
        y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

        x = [None]*2
        z = [None]*2
       
        x[0] = a4*y[0] + a5
        x[1] = a4*y[1] + a5
        z[0] = a6*y[0] + a7
        z[1] = a6*y[1] + a7

        realANS = [None]*3

        if z[0] < 0:
            realANS = numpy.array([x[0],y[0],z[0]])
        elif z[1] < 0:
            realANS = numpy.array([x[1],y[1],z[1]])
        else:
            showError = "Error: height z is zero"
            print(showError)

        print("Method: %d" %method)
        print("FWD_X:%f" %realANS[0])
        print("FWD_Y:%f" %realANS[1])
        print("FWD_Z:%f" %realANS[2])


    return realANS

def DeltaINV(x,y,z):

    # Ultimate radius is 300
    # For safety range 250 is chosen
    maxR = 250

    workingR = math.sqrt(x**2 + y**2)

    rad2deg = 180/math.pi
    deg2rad = math.pi/180

    # working area must be in the area of maxR
    if workingR <= maxR and z > -700 and z <= -350 :
        ########################## parameters ################################
        sb = 315.339
        sp = 88.335
        L = 293         #270
        l = 555         #545
        h = 70
        root3 = math.sqrt(3)
        wb = (root3/6)*sb
        ub = (root3/3)*sb
        wp = (root3/6)*sp
        up = (root3/3)*sp

        a = wb - up
        b = sp/2 - (root3/2)*wb
        c = wp - 0.5*wb

        E1 = 2*L*(y + a)
        F1 = 2*z*L
        G1 = x**2 + y**2 + z**2 + a**2 + L**2 + 2*y*a - l**2

        E2 = -L*(root3*(x+b) + y + c)
        F2 = 2*z*L
        G2 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(x*b + y*c) - l**2

        E3 = L*(root3*(x-b) - y - c)
        F3 = 2*z*L
        G3 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(-x*b + y*c) - l**2;

        t1 = [None]*2
        t2 = [None]*2
        t3 = [None]*2

        t1[0] = (-F1 + math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
        t1[1] = (-F1 - math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
        t2[0] = (-F2 + math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
        t2[1] = (-F2 - math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
        t3[0] = (-F3 + math.sqrt(E3**2 + F3**2 - G3**2)) / (G3 - E3)
        t3[1] = (-F3 - math.sqrt(E3**2 + F3**2 - G3**2)) / (G3 - E3)

        theta1_1 = 2*math.atan(t1[1]);
        theta2_1 = 2*math.atan(t2[1]);
        theta3_1 = 2*math.atan(t3[1]);

        deg1 = theta1_1*rad2deg;
        deg2 = theta2_1*rad2deg;
        deg3 = theta3_1*rad2deg;


        if isinstance(theta1_1, complex) or isinstance(theta2_1, complex) or isinstance(theta3_1, complex):
            print("Error: Driving angle is complex number")

    
        print("INV_deg1:%f" %deg1)
        print("INV_deg2:%f" %deg2)
        print("INV_deg3:%f" %deg3)
        return deg1, deg2, deg3
    
    else:

        PreAng = ReadAngle()
        deg1 = PreAng[0]
        deg2 = PreAng[1]
        deg3 = PreAng[2]
        print("INV_deg1:%f" %deg1)
        print("INV_deg2:%f" %deg2)
        print("INV_deg3:%f" %deg3)
        print("...Out of working range!...")
        return deg1, deg2, deg3

def XYZOutRange(x,y,z):

    if ((x >= 250) or (y >= 250) or (x<=-250) or (y<=-250) or (z>= -350) or (z<= -700)):
        print("---------------------------------------")
        print("---------------------------------------")
        print("-------- Out of safety range!----------")
        print("---------------------------------------")
        print("---------------------------------------")
        WarningFlag = True
    
    else:
        WarningFlag = False

    return WarningFlag


def RunServo(ServoDeg1,ServoDeg2,ServoDeg3):

    ServoDeg1 = 90 + ServoDeg1  # +90 to compensate the mechanical offset setting
    ServoDeg2 = 90 + ServoDeg2
    ServoDeg3 = 90 + ServoDeg3


    servo_ang1 = map(ServoDeg1, 0.0, 360.0, 0, 4095)
    servo_ang2 = map(ServoDeg2, 0.0, 360.0, 0, 4095)
    servo_ang3 = map(ServoDeg3, 0.0, 360.0, 0, 4095)


    dxl1_goal_position = int(servo_ang1)
    dxl2_goal_position = int(servo_ang2)
    dxl3_goal_position = int(servo_ang3)
    servo_PUL = [dxl1_goal_position, dxl2_goal_position, dxl3_goal_position]

    ##### Delta Robot Safety Working Range #####
    if dxl1_goal_position and dxl2_goal_position and dxl3_goal_position > 568:
        dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
        dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
        dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
    else:
        print("ERROR: servo angle is less than 40deg")

def ReadAngle():
    dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION)

    #print("Pul1: %d" %dxl_present_position1)
    #print("Pul2: %d" %dxl_present_position2)
    #print("Pul3: %d" %dxl_present_position3)
    deg1 = map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
    deg2 = map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)
    deg3 = map(dxl_present_position3, 0.0, 4095.0, 0.0, 360.0)

    deg1 = deg1 - 90
    deg2 = deg2 - 90
    deg3 = deg3 - 90
    #print("Read_deg1:%f" %deg1)
    #print("Read_deg2:%f" %deg2)
    #print("Read_deg3:%f" %deg3)

    return deg1, deg2, deg3

def GripperClose():
    servo_com4 = 3600
    dxl4_goal_position = int(servo_com4)
    dxl_comm_result4, dxl_error4 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
    print("Gripper closed")

def GripperOpen():
    servo_com4 = 1500
    dxl4_goal_position = int(servo_com4)
    dxl_comm_result4, dxl_error4 = packetHandler.write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
    print("Gripper Opened")

def TorqueOn():
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)


	print("Torque is enable")

def TorqueOff():
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)


	print("Torque is disable")

def IsMoving1():
    Moving1, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MOVING)
    return Moving1

def IsMoving2():
    Moving2, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MOVING)
    return Moving2

def IsMoving3():
    Moving3, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_MOVING)
    return Moving3

def MovingStatus1():
    MovingStat1, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MOVING_STATUS)
    
    if MovingStat1 > 48:
        print("Motor1 is in Trapezodal Profile")
    elif MovingStat1 < 35 and MovingStat1 > 20:
        print("Motor1 is in Triangular Profile")
    elif MovingStat1 < 20 and MovingStat1 > 3:
        print("Motor1 is in Rectangular Profile")
    elif MovingStat1 < 3:
        print("Motor1 is in Step Mode (No Profile)")
    else:
        print("UNKNOWN Profile...")

    return MovingStat1

def MovingStatus2():
    MovingStat2, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MOVING_STATUS)
    
    if MovingStat2 > 48:
        print("Motor2 is in Trapezodal Profile")
    elif MovingStat2 < 35 and MovingStat2 > 20:
        print("Motor2 is in Triangular Profile")
    elif MovingStat2 < 20 and MovingStat2 > 3:
        print("Motor2 is in Rectangular Profile")
    elif MovingStat2 < 3:
        print("Motor2 is in Step Mode (No Profile)")
    else:
        print("UNKNOWN Profile...")

    return MovingStat2

def MovingStatus3():
    MovingStat3, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_MOVING_STATUS)
    
    if MovingStat3 > 48:
        print("Motor3 is in Trapezodal Profile")
    elif MovingStat3 < 35 and MovingStat3 > 20:
        print("Motor3 is in Triangular Profile")
    elif MovingStat3 < 20 and MovingStat3 > 3:
        print("Motor3 is in Rectangular Profile")
    elif MovingStat3 < 3:
        print("Motor3 is in Step Mode (No Profile)")
    else:
        print("UNKNOWN Profile...")

    return MovingStat3 

def DeltaPos(pre_pos,goal_pos):
    if pre_pos > 360 or goal_pos > 360:
        print("Input position is over than 360!")
        return 0
    else:
        pre_pos_pul = map(pre_pos,0.0,360.0,0.0,4095.0)
        pre_pos_pul = int(pre_pos_pul)
        goal_pos_pul = map(goal_pos,0.0,360.0,0.0,4095.0)
        goal_pos_pul = int(goal_pos_pul)

        delta_pos = abs(goal_pos_pul - pre_pos_pul)

        return delta_pos

def SetProfile1(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 500       # 350 Default                  [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 1: %d" %set_V_PRFL)
    print("A PRFL 1: %d" %set_A_PRFL)
    print("--------------------------------")

def SetProfile2(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 500       # 350 Default                  [0.229RPM]

    #set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
    #set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 2: %d" %set_V_PRFL)
    print("A PRFL 2: %d" %set_A_PRFL)
    print("--------------------------------") 

def SetProfile3(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 500       # 350 Default                  [0.229RPM]

    #set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
    #set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 3: %d" %set_V_PRFL)
    print("A PRFL 3: %d" %set_A_PRFL)
    print("--------------------------------")    

def SetPID1(set_P_Gain,set_I_Gain,set_D_Gain):
    
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
    
    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 1: %d" %position_P_gain)
    print("Position I Gain 1: %d" %position_I_gain)
    print("Position D Gain 1: %d" %position_D_gain)
    print("------------------------------")



def SetPID2(set_P_Gain,set_I_Gain,set_D_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 2: %d" %position_P_gain)
    print("Position I Gain 2: %d" %position_I_gain)
    print("Position D Gain 2: %d" %position_D_gain)
    print("------------------------------")

def SetPID3(set_P_Gain,set_I_Gain,set_D_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 3: %d" %position_P_gain)
    print("Position I Gain 3: %d" %position_I_gain)
    print("Position D Gain 3: %d" %position_D_gain)
    print("------------------------------")

def SetFFGain1(set_FF1_Gain,set_FF2_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

    FF1_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN)
    FF2_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN)

    print("Feedforward 1st Gain 1: %d" %FF1_gain)
    print("Feedforward 2nd Gain 1: %d" %FF2_gain)
    print("------------------------------") 

def SetFFGain2(set_FF1_Gain,set_FF2_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

    FF1_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN)
    FF2_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN)

    print("Feedforward 1st Gain 2: %d" %FF1_gain)
    print("Feedforward 2nd Gain 2: %d" %FF2_gain)
    print("------------------------------")

def SetFFGain3(set_FF1_Gain,set_FF2_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

    FF1_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN)
    FF2_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN)

    print("Feedforward 1st Gain 3: %d" %FF1_gain)
    print("Feedforward 2nd Gain 3: %d" %FF2_gain)
    print("------------------------------")

def TrajectoryGenerationDelta(Vstd,Astd,DelPos1,DelPos2,DelPos3):
    DelPos = [DelPos1, DelPos2, DelPos3]
    MIN = min(DelPos)
    MAX = max(DelPos)
    V_Lim = 180
    A_Lim = 60

    ## If no travel distance for every servos, return a slow speed profile
    if DelPos1 <= 1 and DelPos2 <= 1 and DelPos3 <= 1:
        V1 = 20
        A1 = 5
        V2 = 20
        A2 = 5
        V3 = 20
        A3 = 5
        print("Low travel distance...")
        return V1, A1, V2, A2, V3, A3
    ## If V and A input are over the limit
    if Vstd >= V_Lim or Astd >= A_Lim:
        
        Vstd = V_Lim
        Astd = A_Lim
        
        print("Standard Velocity is over the limit!")
    
    if DelPos1 == MAX:
        V1 = Vstd
        A1 = Astd
        print("Servo1 is the standard")
        t3_1 = 64.0*V1/A1 + 64.0*DelPos1/V1
        t1_std = 64*Vstd/Astd
        t3_tri = 2*t1_std
        t2_1 = 64.0*DelPos1/V1
        t3_std = t3_1
        t2_std = t2_1
        print("t1_std: %f" %t1_std)
        print("t3: %f" %t3_std)
        print("t3_tri: %f" %t3_tri)
        print("DelPos1: %f" %DelPos1)
        print("DelPos2: %f" %DelPos2)
        print("DelPos3: %f" %DelPos3)
        t3_2 = t3_std
        t3_3 = t3_std
        t2_2 = t2_std
        t2_3 = t2_std
        den_std = (t3_std - t2_std)
        V2 = 64.0*DelPos2/t2_2
        V3 = 64.0*DelPos3/t2_3
        A2 = 64*V2 / den_std
        A3 = 64*V3 / den_std
        
    elif DelPos2 == MAX:
        V2 = Vstd
        A2 = Astd
        print("Servo2 is the standard")
        t3_2 = 64.0*V2/A2 + 64.0*DelPos2/V2
        t1_std = 64*Vstd/Astd
        t3_tri = 2*t1_std
        t2_2 = 64.0*DelPos2/V2
        t3_std = t3_2
        t2_std = t2_2
        print("t1_std: %f" %t1_std)
        print("t3: %f" %t3_std)
        print("t3_tri: %f" %t3_tri)
        print("DelPos1: %f" %DelPos1)
        print("DelPos2: %f" %DelPos2)
        print("DelPos3: %f" %DelPos3)
        t3_1 = t3_std
        t3_3 = t3_std  
        t2_1 = t2_std
        t2_3 = t2_std
        den_std = (t3_std - t2_std)       
        V1 = 64.0*DelPos1/t2_1
        V3 = 64.0*DelPos3/t2_3
        A1 = 64*V1 / den_std
        A3 = 64*V3 / den_std

    elif DelPos3 == MAX:
        V3 = Vstd
        A3 = Astd
        print("Servo3 is the standard")
        t3_3 = 64.0*V3/A3 + 64.0*DelPos3/V3
        t1_std = 64*Vstd/Astd
        t3_tri = 2*t1_std
        t2_3 = 64.0*DelPos3/V3
        t3_std = t3_3
        t2_std = t2_3
        print("t1_std: %f" %t1_std)
        print("t3: %f" %t3_std)
        print("t3_tri: %f" %t3_tri)
        print("DelPos1: %f" %DelPos1)
        print("DelPos2: %f" %DelPos2)
        print("DelPos3: %f" %DelPos3)
        t3_1 = t3_std
        t3_2 = t3_std
        t2_1 = t2_std
        t2_2 = t2_std
        den_std = (t3_std - t2_std)       
        V1 = 64.0*DelPos1/t2_1
        V2 = 64.0*DelPos2/t2_2
        A1 = 64*V1 / den_std
        A2 = 64*V2 / den_std

    return V1, A1, V2, A2, V3, A3
    
    


####################################################### Set Servo Configuration #############################################################
# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

ADDR_PRO_CURRENT_LIMIT      = 38
ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_PRESENT_CURRENT    = 126 

ADDR_PRO_OPERATING_MODE     = 11

ADDR_PRO_GOAL_VELOCITY      = 104

ADDR_PRO_ACCELERATION_LIMIT = 40
ADDR_PRO_VELOCITY_LIMIT     = 44
ADDR_PRO_PROFILE_ACCELERATION  = 108
ADDR_PRO_PROFILE_VELOCITY   = 112

ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84
ADDR_PRO_FEEDFORWARD_2nd_GAIN = 88
ADDR_PRO_FEEDFORWARD_1st_GAIN = 90

ADDR_PRO_MOVING             = 122
ADDR_PRO_MOVING_STATUS       = 123


CURRENT_CONTROL                     = 0
POSITION_CONTROL                    = 3 # Default
CURRENT_BASED_POSITION_CONTROL      = 5

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                             # Dynamixel ID: 1
DXL2_ID                      = 2                             # Dynamixel ID: 2
DXL3_ID                      = 3                             # Dynamixel ID: 3
DXL4_ID                      = 4                             # Dynamixel ID: 4

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
'''
# When need to change mode just remove block comment out
# Disable Dynamixel Torque 1

'''


################ Initialize ###################
#TorqueOff()
#SetOperatingMode(POSITION_CONTROL)
### PID Set ###
SetPID1(1500,1000,4000)
SetPID2(1500,1000,4000)
SetPID3(1500,1000,4000)
### Feedforward Set ###
FF1_Gain1 = 0
FF2_Gain1 = 0
SetFFGain1(FF1_Gain1,FF2_Gain1)
FF1_Gain2 = 0
FF2_Gain2 = 0
SetFFGain2(FF1_Gain2,FF2_Gain2)
FF1_Gain3 = 0
FF2_Gain3 = 0
SetFFGain3(FF1_Gain3,FF2_Gain3)
### Goal Current Set ###
SetGoalCurrent(250)
TorqueOn()

DeltaGoHome()

################################################## Get Joy Stick ##############################################

Hats = getHat()
JogDir = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1 

Buttons = getButton()
J1_Btn = Buttons[0] #A
J2_Btn = Buttons[1] #B
J3_Btn = Buttons[2] #X
J4_Btn = Buttons[3] #Y
J5_Btn = Buttons[4] #LB
J6_Btn = Buttons[5] #RB
Back_Btn = Buttons[6] #Back
Start_Btn = Buttons[7] #Start
Memo_Btn = Buttons[8] #Logiccool


################################################################################################################################



waitForStart = True
print("Press Start Button!")

Mem_X = [None]*20
Mem_Y = [None]*20
Mem_Z = [None]*20

runTeach = False

while waitForStart:

    Buttons = getButton()
    Start_Btn = Buttons[7] #Start

    if Start_Btn == 1:
        waitForStart = False
        runTeach = True

    time.sleep(0.1)

print("Dynamic Test Start")

while runTeach:

    TorqueOn()
    print("All Torque is ON")
    time.sleep(1)
    DeltaGoHome()
    time.sleep(1)

    Edge = 150
    Height = -500

    Mem_X[0] = 0
    Mem_Y[0] = 0
    Mem_Z[0] = -400

    Mem_X[1] = 0
    Mem_Y[1] = 0
    Mem_Z[1] = -500

    Mem_X[2] = Edge
    Mem_Y[2] = Edge
    Mem_Z[2] = Height

    Mem_X[3] = -Edge
    Mem_Y[3] = Edge
    Mem_Z[3] = Height

    Mem_X[4] = -Edge
    Mem_Y[4] = -Edge
    Mem_Z[4] = Height

    Mem_X[5] = Edge
    Mem_Y[5] = -Edge
    Mem_Z[5] = Height

    Mem_X[6] = Edge
    Mem_Y[6] = Edge
    Mem_Z[6] = Height

    Mem_X[7] = -Edge
    Mem_Y[7] = Edge
    Mem_Z[7] = Height

    Mem_X[8] = -Edge
    Mem_Y[8] = -Edge
    Mem_Z[8] = Height

    Mem_X[9] = Edge
    Mem_Y[9] = -Edge
    Mem_Z[9] = Height

    Mem_X[10] = 0
    Mem_Y[10] = 0
    Mem_Z[10] = Height


    PreAng = ReadAngle()
    PreAng1 = PreAng[0]
    PreAng2 = PreAng[1]
    PreAng3 = PreAng[2]

    for K in range(0,11):
        startTime = time.time()
        Buttons = getButton()
        Back_Btn = Buttons[6] #Back

        if Back_Btn == 1:
            break

        Mem_ANG = DeltaINV(Mem_X[K],Mem_Y[K],Mem_Z[K])

        GoalPos1 = Mem_ANG[0]
        GoalPos2 = Mem_ANG[1]
        GoalPos3 = Mem_ANG[2]

        DelPos1 = DeltaPos(PreAng1,GoalPos1)
        DelPos2 = DeltaPos(PreAng2,GoalPos2)
        DelPos3 = DeltaPos(PreAng3,GoalPos3)

        VSTD = 40
        ASTD = 8
        ## V = 40, A = 8, time = 0.9~1.0 robot move quite smooth
        ## V = 150, A = 25, time = 0.911 the whole frame not shaking just slightly oscilate on gripper
        ## V = 150, A = 30, time = 0.81~0.86 the robot slightly oscillate
        ## V = 150, A = 35-45, time = 0.71~0.76 the whole frame shaking due to high A

        TRAJ = TrajectoryGenerationDelta(VSTD,ASTD,DelPos1,DelPos2,DelPos3)

        V1 = TRAJ[0]
        A1 = TRAJ[1]
        V2 = TRAJ[2]
        A2 = TRAJ[3]
        V3 = TRAJ[4]
        A3 = TRAJ[5]

        SetProfile1(V1,A1)
        SetProfile2(V2,A2)
        SetProfile3(V3,A3)

        RunServo(GoalPos1,GoalPos2,GoalPos3)
        startTime = time.time()

        print("Move to point %d" %K )

        MoveType1 = MovingStatus1()
        MoveType2 = MovingStatus2()
        MoveType3 = MovingStatus3()
        
        MovingFlag = True
        Block1 = True
        Block2 = True
        Block3 = True

        while MovingFlag:
            Move1 = IsMoving1()
            Move2 = IsMoving2()
            Move3 = IsMoving3()

            if Move1 == 0 and Block1 == True:
                endTime1 = time.time()
                period1 = endTime1 - startTime
                Block1 = False
            if Move2 == 0 and Block2 == True:
                endTime2 = time.time()
                period2 = endTime2 - startTime 
                Block2 = False 
            if Move3 == 0 and Block3 == True:
                endTime3 = time.time()
                period3 = endTime3 - startTime
                Block3 = False

            if Move1 == 0 and Move2 == 0 and Move3 == 0:
                MovingFlag = False
                #endTime = time.time()
                #period = endTime - startTime
                print("Period1: %f" %period1)
                print("Period2: %f" %period2) 
                print("Period3: %f" %period3)                              
                print("Finished point %d" %K)
        
        #time.sleep(1)
        PreAng1 = GoalPos1
        PreAng2 = GoalPos2
        PreAng3 = GoalPos3

    waitForStartAgain = True
    print("Press start for run again")
    print("Press back to exit")
    while waitForStartAgain:
        Buttons = getButton()
        Back_Btn = Buttons[6] #Back
        Start_Btn = Buttons[7] #Start

        if Back_Btn == 1:
            waitForStartAgain = False
            runTeach = False
            #RobotArmGoHome()
            #TorqueOff()

        if Start_Btn == 1:
            K = 0
            waitForStartAgain = False


