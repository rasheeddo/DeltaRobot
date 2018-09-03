# Test servo with Maestro 

import time
import numpy
import math
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def DeltaINV(x,y,z):

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
	t3[0] = (-F3 + math.sqrt(E3**2 + F2**2 - G3**2)) / (G3 - E3)
	t3[1] = (-F3 - math.sqrt(E3**2 + F2**2 - G3**2)) / (G3 - E3)

	theta1 = 2*math.atan(t1[1]);
	theta2 = 2*math.atan(t2[1]);
	theta3 = 2*math.atan(t3[1]);

	deg1 = theta1*rad2deg;
	deg2 = theta2*rad2deg;
	deg3 = theta3*rad2deg;

	ServoDeg1 = 90 + deg1
	ServoDeg2 = 90 + deg2
	ServoDeg3 = 90 + deg3

	if isinstance(theta1, complex) or isinstance(theta2, complex) or isinstance(theta3, complex):
		print("Error: Driving angle is complex number")
		
	return ServoDeg1, ServoDeg2, ServoDeg3

def RunServo(inputDeg1,inputDeg2,inputDeg3):

	pos1 = inputDeg1
	pos2 = inputDeg2
	pos3 = inputDeg3

	servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
	servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
	servo_com3 = map(pos3,0.0,360.0,0.0,4095.0)

	dxl1_goal_position = int(servo_com1)
	dxl2_goal_position = int(servo_com2)
	dxl3_goal_position = int(servo_com3)

	if dxl1_goal_position and dxl2_goal_position and dxl3_goal_position > 568:
		dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
		dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
		dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))
	else:
		print("ERROR: servo angle is less than 40deg")


def ReadAngle():
	dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
	dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
	dxl_present_position3, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION)

	pre_pos1 = map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
	pre_pos2 = map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)
	pre_pos3 = map(dxl_present_position3, 0.0, 4095.0, 0.0, 360.0)

	return pre_pos1,pre_pos2,pre_pos3

def DeltaGoHome():
	pos1 = 90
	pos2 = 90
	pos3 = 90
	################## Go to home position before starting circle ###########################
	servo_ang1 = map(pos1, 0.0, 360.0, 0, 4095)
	servo_ang2 = map(pos2, 0.0, 360.0, 0, 4095)
	servo_ang3 = map(pos3, 0.0, 360.0, 0, 4095)

	dxl1_goal_position = servo_ang1
	dxl2_goal_position = servo_ang2
	dxl3_goal_position = servo_ang3

	dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
	dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
	dxl_comm_result3, dxl_error3 = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))

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

    print("V PRFL 1: %f" %set_V_PRFL)
    print("A PRFL 1: %f" %set_A_PRFL)
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

    print("V PRFL 2: %f" %set_V_PRFL)
    print("A PRFL 2: %f" %set_A_PRFL)
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

    print("V PRFL 3: %f" %set_V_PRFL)
    print("A PRFL 3: %f" %set_A_PRFL)
    print("--------------------------------")    

def TrajectoryGeneration(Vstd,Astd,DelPos1,DelPos2,DelPos3):
    ## Find the shortest moving path ##
    AllDelPos = [DelPos1,DelPos2,DelPos3]
    MIN = min(AllDelPos)
    if DelPos1 == MIN:
        V1 = Vstd
        A1 = Astd
        print("Servo1 is the standard")
        t3_1 = 64*V1/A1 + 64*DelPos1/V1
        t3_2 = t3_1
        t3_3 = t3_1
        t2_1 = 64*DelPos1/V1
        t2_2 = t2_1
        t2_3 = t2_1
        V2 = 64*DelPos2/t2_2
        V3 = 64*DelPos3/t2_3
        A2 = 64*V2 / (t3_2 - 64*DelPos2/V2)
        A3 = 64*V3 / (t3_3 - 64*DelPos3/V3)


    elif DelPos2 == MIN:
        V2 = Vstd
        A2 = Astd
        print("Servo2 is the standard")
        t3_2 = 64*V2/A2 + 64*DelPos2/V2
        t3_1 = t3_2
        t3_3 = t3_2
        t2_2 = 64*DelPos2/V2
        t2_1 = t2_2
        t2_3 = t2_2
        V1 = 64*DelPos1/t2_1
        V3 = 64*DelPos3/t2_3
        A1 = 64*V1 / (t3_1 - 64*DelPos1/V1)
        A3 = 64*V3 / (t3_3 - 64*DelPos3/V3)

    elif DelPos3 == MIN:
        V3 = Vstd
        A3 = Astd
        print("Servo3 is the standard")
        t3_3 = 64*V3/A3 + 64*DelPos3/V3
        t3_1 = t3_3
        t3_2 = t3_3
        t2_3 = 64*DelPos3/V3
        t2_1 = t2_3
        t2_2 = t2_3
        V1 = 64*DelPos1/t2_1
        V2 = 64*DelPos2/t2_2
        A1 = 64*V1 / (t3_1 - 64*DelPos1/V1)
        A2 = 64*V2 / (t3_2 - 64*DelPos2/V2)

    return V1,A1,V2,A2,V3,A3

def TorqueOn():
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

def TorqueOff():
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)


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
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 1 has been successfully connected")
# Disable Dynamixel Torque 2   
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 2 has been successfully connected")
# Disable Dynamixel Torque 3
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel 3 has been successfully connected")

    # Check Operating Mode
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, POSITION_CONTROL)


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
'''

######################### Set PID Gain Position Loop  ##############################
set_P_Gain = 1000    #800 default
set_I_Gain = 20     #0 default
set_D_Gain = 1000   #4700 default

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
print("PID's Gain are set")

position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN)
position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN)
position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN)

print("Position D Gain: %d" %position_D_gain)
print("Position I Gain: %d" %position_I_gain)
print("Position P Gain: %d" %position_P_gain)
print("--------------------------------")


################################################################################################################################


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
########################### Draw circular path ############################
r = 250.0		# Input the radius of a circle 150 is max.
zeta = [None]*360
X = [None]*360
Y1 = [None]*180
Y2 = [None]*180
Y = [None]*360

for i in range(1,361):
    zeta[i-1]=i*deg2rad
    X[i-1] = r*math.cos(zeta[i-1])

for i in range(1,(len(X)/2)+1):
    Y1[i-1] = math.sqrt(r**2 - X[i-1]**2)

j = 0
for i in range((len(X)/2)+1,len(X)+1):
    Y2[j] = -math.sqrt(r**2 - X[j]**2)
    j = j+1

Y = numpy.concatenate((Y1,Y2))


####################### Run Inverse Kinematics ############################
delaySpeed = 0.05
rest = 3

TorqueOn()
SetProfile1(30,10)
SetProfile2(30,10)
SetProfile3(30,10)
DeltaGoHome()

print("Delta is back home")
time.sleep(3)

i = 0
height = -350
increment = 10

while(i<360):

	xx = X[i]
	yy = Y[i]
	zz = height   # home is -485 but actual length is -460 but the difference 

	ServoDEG = DeltaINV(xx,yy,zz)

	ServoANG1 = ServoDEG[0]
	ServoANG2 = ServoDEG[1]
	ServoANG3 = ServoDEG[2] 

	RunServo(ServoANG1,ServoANG2,ServoANG3)

	print("i:%d" %i )
	#print("period: %f" %period)
	print('---------------------------')
	
	if i == 0:
		time.sleep(1)
		# make it delay a bit at first dot
	
	i = i + increment
	time.sleep(delaySpeed)

################### Finish the full circle #####################
xlast = X[359]
ylast = Y[359]
zlast = height   # home is -485 but actual length is -460 but the difference 

ServoDEG = DeltaINV(xlast,ylast,zlast)

ServoANG1 = ServoDEG[0]
ServoANG2 = ServoDEG[1]
ServoANG3 = ServoDEG[2] 

RunServo(ServoANG1,ServoANG2,ServoANG3)

time.sleep(1)

DeltaGoHome()

