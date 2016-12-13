from __future__ import division
import numpy as np
import math
import unittest
from math import sin
from math import cos
import math


class JointSpaceVector(object):

    def __init__(self, angles=[0,0,0,0,0,0]):
        self.angles = angles
        self.DH_PARAM_D = [-815,        0,              0,          -1545,      0,          0, -158]   #d
        self.DH_PARAM_A = [350,         1200,           145,        0,          0,          0, 0]      #a
        self.DH_PARAM_ALPHA  = [math.radians(-90),    0,              math.radians(-90),   math.radians(90),    math.radians(-90),   0, 0]      #Alpha



    def t01(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[0], a=self.DH_PARAM_A[0], theta=self.angles[0], d=self.DH_PARAM_D[0])

    def t12(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[1], a=self.DH_PARAM_A[1], theta=self.angles[1]+math.radians(90), d=self.DH_PARAM_D[1])

    def t23(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[2], a=self.DH_PARAM_A[2], theta=self.angles[2], d=self.DH_PARAM_D[2])

    def t34(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[3], a=self.DH_PARAM_A[3], theta=self.angles[3], d=self.DH_PARAM_D[3])

    # split up t34 as tz * rotz * tx * rotx as t3WRIST * tWRIST4
    def t3WRIST(self):
        return Transformation(alpha=0, a=0, theta=0, d=self.DH_PARAM_D[3])

    def tWRIST4(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[3], a=self.DH_PARAM_A[3], theta=self.angles[3], d=0)

    def t45(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[4], a=self.DH_PARAM_A[4], theta=self.angles[4], d=self.DH_PARAM_D[4])
    
    def t56(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[5], a=self.DH_PARAM_A[5], theta=self.angles[5], d=self.DH_PARAM_D[5])

    def t6TCP(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[6], a=self.DH_PARAM_A[6], theta=0, d=self.DH_PARAM_D[6])

    def baseToTCP(self):
        return self.t01() * self.t12() * self.t23() * self.t34() * self.t45() * self.t56() * self.t6TCP()

    def baseToWrist(self):
        return self.t01() * self.t12() * self.t23() * self.t3WRIST()

    def wristToPose(self):
        return self.tWRIST4() * self.t45() * self.t56()
    
    def __str__(self):
        return "(" + ", ".join(["%.2f" % a for a in self.angles]) + ")"



class Transformation(object):
    """A simple class wrapping a standart robot transformation"""
    def __init__(self, matrix=np.eye(4,4,dtype=np.float64), a=0, d=0, alpha=0, theta=0):
        self.matrix = matrix
        self.rotate_x(alpha) # because we start from the right side
        self.translate([a, 0, 0])
        self.rotate_z(theta)
        self.translate([0, 0, d])

    def rotate(self, axis, angle="0"):
        """ rotate given translation by given axis and angle """
        if axis == "x":
            return self.rotate_x
        if axis == "y":
            return self.rotate_y
        if axis == "z":
            return self.rotate_z
        return self

    def transform(self, matrix):
        """ transform self by given matrix (transformation*matrix) """
        self.matrix = np.dot(matrix, self.matrix)
        return self

    def translate(self, vector3):
        """ translate matrix by given vector3 """
        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][3] = vector3[0]
        transform[1][3] = vector3[1]
        transform[2][3] = vector3[2]
        return self.transform(transform)


    def rotate_x(self, angle):
        """ rotate around the x axis """
        transform = np.eye(4, 4,dtype=np.float64)
        transform[1][1] = cos(angle)
        transform[1][2] = -1 * sin(angle)
        transform[2][1] = sin(angle)
        transform[2][2] = cos(angle)
        return self.transform(transform)


    def rotate_y(self, angle):
        """ rotate around the y axis """
        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][0] = cos(angle)
        transform[0][2] = -1 * sin(angle)
        transform[2][0] = sin(angle)
        transform[2][2] = cos(angle)
        return self.transform(transform)


    def rotate_z(self, angle):
        """ rotate around the z axis """
        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][0] = cos(angle)
        transform[0][1] = -1 * sin(angle)
        transform[1][0] = sin(angle)
        transform[1][1] = cos(angle)
        return self.transform(transform)

    def __mul__(self, other):
        return Transformation(matrix=np.dot(self.matrix, other.matrix))

    def position(self):
        """ return the positional part of the transformation as a vector """
        beta = math.atan2(self.matrix[2][0], math.sqrt(self.matrix[0][0]**2 + self.matrix[1,0]**2))
        cbeta = math.cos(beta)
        alpha = math.atan2(self.matrix[1][0] / cbeta, self.matrix[0][0] / cbeta)
        gamma = math.atan2(self.matrix[2][1] / cbeta, self.matrix[2][2] / cbeta)
        return ( self.matrix[0][3], self.matrix[1][3], self.matrix[2][3], alpha, beta, gamma )

    def __str__(self):
        return str(self.matrix)
    
    def cart_values(self):
        cv = "%.2f;" % self.position()[0]
        cv = cv + "%.2f;" % self.position()[1]
        cv = cv + "%.2f;" % self.position()[2]
        cv = cv + "%.2f;" % np.rad2deg(self.position()[3])
        cv = cv + "%.2f;" % np.rad2deg(self.position()[4])
        cv = cv + "%.2f;" % np.rad2deg(self.position()[5])
        print(cv)
        return cv

    def inv(self):
        inverse = np.linalg.inv(self.matrix)
        return Transformation(matrix=inverse)

    def getWristJoints(self):
        theta5 = -1 * np.arccos(self.matrix[2][2])
# handle wrist singularity
# just rotate the point a tiny little bit away, to get out of the singularity
# the error we introduce is somewhere close to 1e-5 deg
        if(theta5 == 0):
            self.rotate_x(0.0000001)
            self.rotate_y(0.0000001)
            self.rotate_z(0.0000001)

        theta5 = -1 * np.arccos(self.matrix[2][2])
        theta4 = np.arctan2(self.matrix[1][2], self.matrix[0][2])
        theta6 = np.arctan2(self.matrix[2][1], -1 * self.matrix[2][0])
        solution1 = (theta4, theta5, theta6)
# you get solution 2 by inverting solution 1
        solution2 = (theta4+math.pi, -1 * theta5, theta6 + math.pi)
        return [solution1, solution2]

    def getJointVectors(self):
        """computes the inverse kinematics for this transformation"""
        wcp = self * JointSpaceVector().t6TCP().inv()
        arr = wcp.position()[0:3]
        position_solutions = calcAngles(arr)
        ret = []
        for p in position_solutions:
            jsv = JointSpaceVector(angles=p+(0,0,0))
            wristTransformation = jsv.baseToWrist().inv() * self * jsv.t6TCP().inv()
            wristJoints = wristTransformation.getWristJoints()
            ret = ret + [JointSpaceVector(angles=p+wristJoints[0])]
            ret = ret + [JointSpaceVector(angles=p+wristJoints[1])]
        return ret

    def getIKSolutions(self):
        return self.getJointVectors()


def calcAngles(arr):
    """ arr: position vector """
    #calc Rz
    #calc P
    # -> calc wc
    #w_c = getWc(p, r)
    a1 = getPhy1(arr[0],arr[1])
    a2 = getPhy2(arr[0],arr[1],arr[2])
    a3 = getPhy3(arr[0],arr[1],arr[2])
    
    s1 = (a1[0], a2[0], a3[0])
    s2 = (a1[0], a2[1], a3[1])
    s3 = (a1[1], a2[2], a3[2])
    s4 = (a1[1], a2[3], a3[3])

    return [s1, s2, s3, s4]

DH_PARAM_D = JointSpaceVector().DH_PARAM_D
DH_PARAM_A = JointSpaceVector().DH_PARAM_A
DH_PARAM_ALPHA = JointSpaceVector().DH_PARAM_ALPHA

def getPhy1(x,y):
    phy1_sol1 = math.atan2(y, x)                #front elbow
    phy1_sol2 = math.atan2(y, x) + math.pi      #rear elbow
    return [phy1_sol1, phy1_sol2]

def getPhy2(x, y, z):
    xy = getHypot(x,y)
    d34 = getHypot(DH_PARAM_A[2],DH_PARAM_D[3])
    dFront = getDFront(xy,z)
    dRear = getDRear(xy,z)
    gamma = math.atan((z - DH_PARAM_D[0]) / (xy - DH_PARAM_A[0]))
    betaFront = math.acos((-(d34)**2 + (DH_PARAM_A[1]**2) + (dFront**2)) / (2.0 * DH_PARAM_A[1] * dFront))
    betaRear = math.acos((-(d34)**2 + (DH_PARAM_A[1]**2) + (dRear**2)) / (2.0 * DH_PARAM_A[1] * dRear))
                     
    phy2_sol1 = (betaFront + gamma) - (math.pi/2.0)       #front elbow up
    phy2_sol2 = (gamma - betaFront) - (math.pi/2.0)       #front elbow down
    phy2_sol3 = math.pi/2.0 - (betaRear + gamma)          #rear elbow up
    phy2_sol4 = math.pi/2.0 - (gamma - betaRear)          #rear elbow down
    return [phy2_sol1, phy2_sol2, phy2_sol3, phy2_sol4]

def getPhy3(x, y, z):
    xy = getHypot(x,y)
    d34 = getHypot(DH_PARAM_A[2],DH_PARAM_D[3])
    dFront = getDFront(xy,z)
    dRear = getDRear(xy,z)
    delta = math.atan(DH_PARAM_D[3] / DH_PARAM_A[2])
    epsilonFront = math.acos((-(dFront)**2 + (d34**2) + (DH_PARAM_A[1]**2)) / (2.0 * d34 * DH_PARAM_A[1]))
    epsilonRear = math.acos((-(dRear)**2 + (d34**2) + (DH_PARAM_A[1]**2)) / (2.0 * d34 * DH_PARAM_A[1]))
    
    phy3_sol1 = delta + epsilonFront - math.pi                          #front elbow up
    phy3_sol2 = math.pi - (epsilonFront - delta)                        #front elbow down
    phy3_sol3 = (math.pi/2 - (epsilonRear - delta)) + (math.pi/2.0)     #rear elbow up
    phy3_sol4 = math.pi/2 - ((1.5 * math.pi) - delta - epsilonRear)     #rear elbow down
    return [phy3_sol1, phy3_sol2, phy3_sol3, phy3_sol4]

def getDFront(xy, z):
    d = math.sqrt((xy - DH_PARAM_A[0])**2 + (z - DH_PARAM_D[0])**2)
    return d

def getDRear(xy, z):
    d = math.sqrt((xy + DH_PARAM_A[0])**2 + (z - DH_PARAM_D[0])**2)
    return d

def getHypot(a, b):
    return math.sqrt(a**2 + b**2)



class TestDirectKinematics(unittest.TestCase):
    def testNullTransformation(self):
        nullTransformation=JointSpaceVector(angles=[0,-90,-90,0,0,0])
        nullTransformation.DH_PARAM_A=[260, 680, -35, 0, 0, 0, 0]
        nullTransformation.DH_PARAM_D=[-675, 0, 0, -670, 0, -115, -670]
        nullTransformation.DH_PARAM_ALPHA=[90, 0, 90, -90, 90, 0, 0]
        desired_result = np.array([
            [.0,.0,-1.0,1725.0],
            [.0,-1.0,.0,.0],
            [-1.0,.0,.0,-640.0],
            [.0,.0,.0,1.0]],dtype=np.float64)
        np.testing.assert_allclose(nullTransformation.baseToTCP().matrix, desired_result, atol=0.001)

    def testStandartTransformation(self):
        nullTransformation=JointSpaceVector(angles=[0,-90,0,0,0,0])
        nullTransformation.DH_PARAM_A=[260, 680, -35, 0, 0, 0, 0]
        nullTransformation.DH_PARAM_D=[-675, 0, 0, -670, 0, -115, -670]
        nullTransformation.DH_PARAM_ALPHA=[90, 0, 90, -90, 90, 0, 0]
        desired_result = np.array([
            [.0,.0,-1.0,1045.0],
            [.0,-1.0,.0,.0],
            [-1.0,.0,.0,-1320.0],
            [.0,.0,.0,1.0]],dtype=np.float64)
        np.testing.assert_allclose(nullTransformation.baseToTCP().matrix, desired_result, atol=0.001)

    def testStraightTransformation(self):
        nullTransformation=JointSpaceVector(angles=[0,-90,-90,-90,0,0])
        nullTransformation.DH_PARAM_A=[260, 680, -35, 0, 0, 0, 0]
        nullTransformation.DH_PARAM_D=[-675, 0, 0, -670, 0, -115, -670]
        nullTransformation.DH_PARAM_ALPHA=[90, 0, 90, -90, 90, 0, 0]
        desired_result = np.array([
            [.0,-1.0,.0,295.0],
            [1.0,.0,.0,.0],
            [.0,.0,1.0,-2140.0],
            [.0,.0,.0,1.0]],dtype=np.float64)
        np.testing.assert_allclose(nullTransformation.baseToTCP().matrix, desired_result, atol=0.001)



if __name__ == '__main__':
    pass
    #unittest.main()
