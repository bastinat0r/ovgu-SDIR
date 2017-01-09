from openravepy import *
import Kinematics as kin
import MotionFunctions as mf
import numpy as np
import math
import sys
import socket
from RobotControl import *


# handles the data transfer between openrave (server) and the GUI (client)
def dataTransfer():
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#      
#     try:
#         s.bind(('localhost', 54321))
#         while True:
#             data, addr = s.recvfrom(2048)   
#             send = handleData(data)
#             # send new informations to the GUI for updating purposes
#             s.sendto(send, addr)
#     except:
#         while True:
#             data, addr = s.recvfrom(2048)   
#             send = handleData(data)
#             # send new informations to the GUI for updating purposes
#             s.sendto(send, addr)
#     finally:
#         s.close()
        
    UDP_IP = "localhost"
    UDP_PORT = 54321
      
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind((UDP_IP, UDP_PORT))
        while True:
            data, addr = sock.recvfrom(2048)
            send = handleData(data) 
            sock.sendto(send, addr)
    finally:
        sock.close()
    

def assembleValueString():
    # prefix for parsing
    prefix = "VAL#"
    # get Axis values
    axis_arr = robot.GetDOFValues()
    # convert to string
    axis_values = ";".join(["%.2f"%np.rad2deg(a) for a in axis_arr]) + "#"
    tcp = rc.GetTCPTransformation()
    cart_values = tcp.cart_values()
    return prefix+axis_values+cart_values
    

# handles the data received from the GUI and sets up data for sending
def handleData(data):
    # split data string
    data_arr = data.split("#")
    
    # check if GUI requests the current robot axis values as well as current orientation and position 
    if data_arr[0] == 'GET':
        return assembleValueString()
    
    elif data_arr[0] == 'MOV':
        # get values
        values = data_arr[1].split(';')
        # convert from string to float and save in numpy array
        target = np.array([math.radians(float(values[0])), math.radians(float(values[1])), math.radians(float(values[2])), math.radians(float(values[3])), math.radians(float(values[4])), math.radians(float(values[5]))])
                
        # get the motion type
        motion_type = data_arr[2]
        
        # get trajectory
        if motion_type == 'L':
            trajectory = mf.LINtoConfiguration(robot.GetDOFValues(), target)
        else:
            trajectory = mf.PTPtoConfiguration(robot.GetDOFValues(), target, motion_type)
        # move robot
        mf.Move(robot, trajectory)
        
        # send new information about the robot's axis values, position and orientation to the GUI for updating purpose
        # prefix for parsing
        return assembleValueString()
    
    # check if inverse kinematics should be calculated
    if data_arr[0] == "CAL":
        # get string with values
        values = data_arr[1].split(';')
        print(values)
        
        # calculate inverse kinematic solution
        trans = kin.Transformation()
        trans.rotate_x(np.deg2rad(float(values[3])))
        trans.rotate_y(np.deg2rad(float(values[4])))
        trans.rotate_z(np.deg2rad(float(values[5])))
        trans.translate((float(values[0]), float(values[1]), float(values[2])))
        print(str(trans))
        print([str(a)for a in trans.getIKSolutions()])        
        
        # send the (multiple) solutions to the GUI
        # prefix for parsing
        prefix = "INK#"
        # adding dummy values (you need to replace them with the solutions)
        ik_values = [str(iks) for iks in trans.getIKSolutions()]
        return prefix+"\n".join(ik_values)
    
    
if __name__ == "__main__":
    # setting up the operave environment
    env = Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    env.Load('../../MyData/MyEnvironment/MyEnv.xml') # load a simple scene
    robot = env.GetRobots()[0] # get the first robot
    rc = RobotControl(robot)
    mf.setLimits(robot)
