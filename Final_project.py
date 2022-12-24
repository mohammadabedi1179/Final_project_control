import serial
import os
from dynamixel_sdk import *
from itertools import count
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


ADDR_MX_TORQUE_ENABLE      = 24               
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4
PROTOCOL_VERSION            = 1.0
DXL1_ID                     = 1                 
DXL2_ID                     = 2                 
BAUDRATE                    = 1000000           
DEVICENAME                  = '/dev/ttyUSB1'    
TORQUE_ENABLE               = 1           
TORQUE_DISABLE              = 0           
DXL_MINIMUM_POSITION_VALUE  = 0           
DXL_MAXIMUM_POSITION_VALUE  = 500
DXL_MOVING_STATUS_THRESHOLD = 20
ADDR_MX_MOVING_SPEED = 32
LEN_MX_MOVING_SPEED = 4                

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
groupSyncWrite_T = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED)
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

ser = serial.Serial('/dev/ttyUSB0',baudrate=9600)
position_1 = []
position_2 = []
while 1:
    if ser.isOpen():
        #ser.reset_input_buffer()
        line = ser.readline(9)
        num = line.strip().decode("utf-8", errors='replace')
        outputs = []
        sp = num.split(" ")
        try:
            outputs = [float(i) for i in sp]
            position_1.append(outputs[0])
            position_2.append(outputs[1])
            goal_position_2 = int(outputs[1]/180*1023 + 330)
            goal_position_1 = int(outputs[0]/180*1023 + 250)
            if goal_position_1 > 501 and goal_position_2 > 561:
                goal_position_1 = 500
                goal_position_2 = 560
            print(outputs)
            param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(goal_position_1)), DXL_HIBYTE(DXL_LOWORD(goal_position_1)), DXL_LOBYTE(DXL_HIWORD(goal_position_1)), DXL_HIBYTE(DXL_HIWORD(goal_position_1))]
            param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(goal_position_2)), DXL_HIBYTE(DXL_LOWORD(goal_position_2)), DXL_LOBYTE(DXL_HIWORD(goal_position_2)), DXL_HIBYTE(DXL_HIWORD(goal_position_2))]

            dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position_1)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
                quit()
            dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position_2)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
                quit()
            dxl_comm_result = groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            groupSyncWrite.clearParam()


            dxl_addparam_result = groupSyncWrite_T.addParam(DXL1_ID, 800)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
                quit()
            dxl_addparam_result = groupSyncWrite_T.addParam(DXL2_ID, 800)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
                quit()
            dxl_comm_result = groupSyncWrite_T.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            groupSyncWrite_T.clearParam()

                
        except:
            pass

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

portHandler.closePort()
ser.close()