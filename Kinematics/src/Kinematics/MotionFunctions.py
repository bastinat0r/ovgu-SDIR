from __future__ import division
from math import sqrt
import numpy as np
import time
import Kinematics as kin

#maximum rotation of each join
#===============================================================================
# MAX_ROT_J = [+-185,
#              +35/-135,
#              +158/-120,
#              +-350,
#              +-130,
#              +-350]
#===============================================================================

#maximum velocity of each join
#===============================================================================
# MAX_VEL_J = [100.0,
#              80.0,
#              80.0,
#              230.0,
#              165.0,
#              249.0]
#===============================================================================

#maximum acceleration of each joint
#===============================================================================
# MAX_ACC_J = [250.0,
#              250.0,
#              250.0,
#              410.0,
#              410.0,
#              660.0]
#===============================================================================

MAX_ROT_J = np.empty([2,6])
MAX_VEL_J = np.empty(6)
MAX_ACC_J = np.empty(6)

deltaT = 1

def setLimits(robot):
    global MAX_ACC_J
    global MAX_VEL_J
    global MAX_ROT_J
    MAX_ACC_J = robot.GetDOFAccelerationLimits()
    MAX_VEL_J = robot.GetDOFVelocityLimits()
    MAX_ROT_J = robot.GetDOFLimits()

def setDelta(delta):
    global deltaT
    deltaT = delta
    
def LINtoConfiguration(start_cfg, target_cfg):
    vMax = 500
    aMax = 300
    
    trajectory_cart = np.empty([100, 6])
    
    target_cfg = getEndPoints(target_cfg)
    
    jsv_start = kin.JointSpaceVector(start_cfg)
    jsv_end = kin.JointSpaceVector(target_cfg)
    start_cfg_cart = jsv_start.baseToTCP().position()
    target_cfg_cart = jsv_end.baseToTCP().position()
    
    diff = np.subtract(target_cfg_cart, start_cfg_cart)
    dist = getDistanceL(diff)
    if dist == 0:
        return np.array([start_cfg, target_cfg])
    
    velProf = getVelProfileL(dist,aMax,vMax)
    
    delta = velProf[2] / 100.0
    setDelta(delta)
    
    #calculate cartesian trajectory
    for i in xrange(100):
        tTmp = delta * (i+1)
        tAcc = velProf[0]
        tFlat = velProf[1]
        tAll = velProf[2]
        
        if tTmp == 0:
            trajectory_cart[i] = start_cfg_cart
        elif tTmp > 0 and tTmp <= tAcc:
            sTmp = getDistFromConstAcc(tTmp, aMax)
            delta_s = sTmp / dist
            trajectory_cart[i] = getPosFromDistL(start_cfg_cart, delta_s, diff)
        elif tTmp > tAcc and tTmp <= tAcc + tFlat:
            sTmp = getDistFromConstAcc(tAcc, aMax) + getDistFromConstVel(tTmp - tAcc, vMax)
            delta_s = sTmp / dist
            trajectory_cart[i] = getPosFromDistL(start_cfg_cart, delta_s, diff)
        elif tTmp > tAcc + tFlat and tTmp < tAll:
            sTmp = getDistFromConstAcc(tAcc, aMax) + getDistFromConstVel(tFlat, vMax) + (getDistFromConstVel(tTmp-(tFlat+tAcc), vMax) - getDistFromConstAcc(tTmp-(tFlat+tAcc), vMax))
            delta_s = sTmp / dist
            trajectory_cart[i] = getPosFromDistL(start_cfg_cart, delta_s, diff)
        else:
            trajectory_cart[i] = target_cfg_cart
                   
    trajectory = getConfiguration(start_cfg, target_cfg, getJointSets(trajectory_cart))        
    
    return trajectory

#calculate cartesian velocity profile
def getVelProfileL(dist, a, v):    
    if dist != 0:
        tAcc = getTimeFromConstAcc(v, a)
        sAcc = getDistFromConstAcc(tAcc, a)
    
        sFlat = dist - sAcc * 2.0
    
        if sFlat < 0:
            #in case of triangle
            vTmp = getVelFromConstAcc(dist, a)
            tAcc = getTimeFromConstAcc(vTmp, a)
            tFlat = 0.0
        else:
            #in case of trapazoid
            tFlat = getTimeFromConstVel(sFlat, v)
        
        tAll = tAcc * 2.0 + tFlat
    else:
        tAcc = 0.0
        tFlat = 0.0
        tAll = 0.0
    return [tAcc, tFlat, tAll]

#calculate cartesian distance
def getDistanceL(vec):
    return sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)

#calculate cartesian position between points
def getPosFromDistL(start, delta_s, diff):
    return np.multiply(diff, delta_s) + start

#get inverse kinematics
def getJointSets(traj_cart):
    traj_joints = []
    for i in xrange(100):
        c_set = traj_cart[i]
        j_set = []
        # calculate inverse kinematic solution
        trans = kin.Transformation()
        trans.rotate_z(float(c_set[5]))
        trans.rotate_y(float(c_set[4]))
        trans.rotate_x(float(c_set[3]))
        trans.translate((float(c_set[0]), float(c_set[1]), float(c_set[2])))
        for x in trans.getIKSolutions():
            j_set.append(list(x.angles))
        traj_joints.append(j_set)   
    return traj_joints
    

#calculate joint configurations from cartesian trajectory
def getConfiguration(start_cfg, target_cfg, trajectory_joints):
    trajectory = np.empty([100, 6])
    lastConf = start_cfg
    for i in xrange(100):
        if i > 98:
            trajectory[99] = target_cfg
        else:
            if np.shape(trajectory_joints[i])[0] > 0:
                lastConf = trajectory_joints[i][getClosestNextConf(lastConf,trajectory_joints[i])]
            trajectory[i] = lastConf
    return trajectory

#calculates the nearest configuration for the next timeslot
def getClosestNextConf(c_start, joint_set):
    length = np.shape(joint_set)[0]
    distances = np.empty(length)
    if length > 1:
        for i in xrange(length):
            dist_i = (np.abs(np.subtract(joint_set[i],c_start)))
            distances[i] = np.sum(dist_i) 
            return np.argmin(distances)
    else:
        return 0



def PTPtoConfiguration(start_cfg, target_cfg, motiontype):
    """PTP path planning
    
    :param start_cfg: Current axis angle of the robot
    :type start_cfg: array of floats
    :param target_cfg: Target angle of the robot
    :type target_cfg: array of floats
    :param motiontype: Type of motion (asynchronous, synchronous, fully synchronous)
    :type motiontype: int
    :returns: Array containing the axis angles of the interpolated path
    :rtype: matrix of floats
    """
    
    trajectory = np.empty([100, 6])

    target_cfg = getEndPoints(target_cfg)

    diff = target_cfg - start_cfg
    dist = np.abs(diff)
    
    #get asynchronous velocity profile
    velProf = getVelProfile(dist,MAX_VEL_J)
    
    j_tMax = getTMax(velProf)
    tMax = velProf[j_tMax][2]
    
    if motiontype == 'S':
        #get synchronous velocity profile
        velProf = getSynchVelProfile(dist,[j_tMax,tMax])
    
    delta = tMax / 100.0
    setDelta(delta)
    
    #calculate trajectory
    for i in xrange(100):
        tTmp = delta * i
        for j in xrange(6):
            tAcc = velProf[j][0]
            tFlat = velProf[j][1]
            tAll = velProf[j][2]
            aTmp = MAX_ACC_J[j]
            vTmp = tAcc * aTmp
            
            if tTmp == 0:
                trajectory[i][j] = start_cfg[j]
            elif tTmp > 0 and tTmp <= tAcc:
                sTmp = getDistFromConstAcc(tTmp, aTmp)
                trajectory[i][j] = getPosFromDist(start_cfg[j], sTmp, diff[j])
            elif tTmp > tAcc and tTmp <= tAcc + tFlat:
                sTmp = getDistFromConstAcc(tAcc, aTmp) + getDistFromConstVel(tTmp - tAcc, vTmp)
                trajectory[i][j] = getPosFromDist(start_cfg[j], sTmp, diff[j])
            elif tTmp > tAcc + tFlat and tTmp <= tAll:
                sTmp = getDistFromConstAcc(tAcc, aTmp) + getDistFromConstVel(tFlat, vTmp) + (getDistFromConstVel(tTmp-(tFlat+tAcc), vTmp) - getDistFromConstAcc(tTmp-(tFlat+tAcc), aTmp))
                trajectory[i][j] = getPosFromDist(start_cfg[j], sTmp, diff[j])
            else:
                trajectory[i][j] = target_cfg[j]
    
    return trajectory

#returns new endpoints depending on the joint limits
def getEndPoints(endpoints):
    newPoints = np.empty(6)
    for i in xrange(6):
        if MAX_ROT_J[0][i] <= endpoints[i] <= MAX_ROT_J[1][i]:
            newPoints[i] = endpoints[i]
        else:
            newPoints[i] = MAX_ROT_J[1][i] if endpoints[i] >= 0 else MAX_ROT_J[0][i]
    return newPoints

#returns timeslots of the velocity profile
def getVelProfile(dist,velArr):
    velProfile = np.empty([6, 3])
    
    for i in xrange(6):
        if dist[i] != 0:
            tAcc = getTimeFromConstAcc(velArr[i], MAX_ACC_J[i])
            sAcc = getDistFromConstAcc(tAcc, MAX_ACC_J[i])
        
            sFlat = dist[i] - sAcc * 2.0
        
            if sFlat < 0:
                #in case of triangle
                vTmp = getVelFromConstAcc(dist[i], MAX_ACC_J[i])
                tAcc = getTimeFromConstAcc(vTmp, MAX_ACC_J[i])
                tFlat = 0.0
            else:
                #in case of trapazoid
                tFlat = getTimeFromConstVel(sFlat, velArr[i])
            
            tAll = tAcc * 2.0 + tFlat
        else:
            tAcc = 0.0
            tFlat = 0.0
            tAll = 0.0
        velProfile[i] = [tAcc,tFlat,tAll]
    
    return velProfile

#returns a synchronized velocity profile
def getSynchVelProfile(dist, arrTmax):
    velArr = np.empty(6)
    for i in xrange(6):
        if dist[i] == 0 or i == arrTmax[0]:
            velArr[i] = MAX_VEL_J[i]
        else: 
            velArr[i] = getVMaxFromTime(dist, i, arrTmax[1])
    return getVelProfile(dist,velArr)

#returns index with maximum value of tAll
def getTMax(arr):
    tmpMax = np.argmax(arr, axis=0)
    return tmpMax[2]

#basic physics
def getTimeFromConstAcc(v, a):
    return v / a

#basic physics
def getDistFromConstAcc(t, a):
    return (a * t**2.0) / 2.0

#basic physics
def getTimeFromConstVel(s, v):
    return s / v

#basic physics
def getDistFromConstVel(t, v):
    return t * v

#basic physics
def getVelFromConstAcc(s, a):
    return sqrt(s * a)

#calculates new maximum velocity
def getVMaxFromTime(dist, index, tMax):
    tmpCalc = (tMax * MAX_ACC_J[index]) / 2.0
    vMax = tmpCalc + sqrt((tmpCalc ** 2.0) - (MAX_ACC_J[index] * dist[index]))
    if vMax < 0 or vMax > MAX_VEL_J[index]:
        vMax = tmpCalc - sqrt((tmpCalc ** 2.0) - (MAX_ACC_J[index] * dist[index]))
    return vMax

#calculates exact position between start and endpoint
def getPosFromDist(start, s, diff):
    return (diff / abs(diff)) * s + start

#move robot
def Move(robot, trajectory):
    #with robot.GetEnv():
        for i in range(trajectory.shape[0]):
            with robot.GetEnv():
                robot.SetDOFValues(trajectory[i])
            time.sleep(deltaT)

