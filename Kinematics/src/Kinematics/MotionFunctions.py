from __future__ import division
from math import sqrt
import numpy as np
import time

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

MAX_VEL_J = np.array(6)
MAX_ACC_J = np.array(6)

def setLimits(robot):
    global MAX_ACC_J
    global MAX_VEL_J
    MAX_ACC_J = robot.GetDOFAccelerationLimits()
    MAX_VEL_J = robot.GetDOFVelocityLimits()

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

    diff = target_cfg - start_cfg
    dist = np.abs(diff)
    
    velProf = getVelProfile(dist,MAX_VEL_J)
    
    j_tMax = getTMax(velProf)
    tMax = velProf[j_tMax][2]
    
    if motiontype == 'S':
        velProf = getSynchVelProfile(dist,[j_tMax,tMax])
    
    delta = tMax / 100.0  #/spec rate
    
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
    for i in range(trajectory.shape[0]):
        robot.SetDOFValues(trajectory[i])
        time.sleep(0.01)
