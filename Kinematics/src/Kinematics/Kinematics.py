import numpy as np
import math
import unittest



def cosd(angle):
    """cosine of angle in degree"""
    return math.cos(math.radians(angle))

def sind(angle):
    """sine of angle in degree"""
    return math.sin(math.radians(angle))

class JointSpaceVector(object):

    def __init__(self, angles=[0,0,0,0,0,0]):
        self.angles = angles
        self.DH_PARAM_D = [-815,        0,              0,          -1545,      0,          -158]   #d
        self.DH_PARAM_A = [350,         1200,           145,        0,          0,          0]      #a
        self.DH_PARAM_ALPHA  = [-90,    0,              -90,   90,    -90,   0]      #Alpha



    def t01(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[0], a=self.DH_PARAM_A[0], theta=self.angles[0], d=self.DH_PARAM_D[0])

    def t12(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[1], a=self.DH_PARAM_A[1], theta=self.angles[1]+90, d=self.DH_PARAM_D[1])

    def t23(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[2], a=self.DH_PARAM_A[2], theta=self.angles[2], d=self.DH_PARAM_D[2])

    def t34(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[3], a=self.DH_PARAM_A[3], theta=self.angles[3], d=self.DH_PARAM_D[3])

    def t45(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[4], a=self.DH_PARAM_A[4], theta=self.angles[4], d=self.DH_PARAM_D[4])
    
    def t5TCP(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[5], a=self.DH_PARAM_A[5], theta=self.angles[5], d=self.DH_PARAM_D[5])

    def t3WCP(self):
        return Transformation(alpha=self.DH_PARAM_ALPHA[6], a=self.DH_PARAM_A[6], theta=0,              d=self.DH_PARAM_D[6])



    def wristCenterPointTransformation(self):
        return self.t01() * self.t12() * self.t23() * self.t3WCP()

    def wristToTCPTransformation(self):
        return self.t34() * self.t45() * self.t5TCP()

    def baseToTCP(self):
        return self.t01() * self.t12() * self.t23() * self.t34() * self.t45() * self.t5TCP()


class Transformation(object):
    """A simple class wrapping a standart robot transformation"""
    def __init__(self, matrix=np.eye(4,4,dtype=np.float64), a=0, d=0, alpha=0, theta=0):
        self.matrix = matrix
        self.rotate_x(alpha) # because we start from the right side
        self.translate([a, 0, d])
        self.rotate_z(theta)

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
        transform[1][1] = cosd(angle)
        transform[1][2] = -1 * sind(angle)
        transform[2][1] = sind(angle)
        transform[2][2] = cosd(angle)
        return self.transform(transform)


    def rotate_y(self, angle):
        """ rotate around the y axis """
        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][0] = cosd(angle)
        transform[0][2] = -1 * sind(angle)
        transform[2][0] = sind(angle)
        transform[2][2] = cosd(angle)
        return self.transform(transform)


    def rotate_z(self, angle):
        """ rotate around the z axis """
        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][0] = cosd(angle)
        transform[0][1] = -1 * sind(angle)
        transform[1][0] = sind(angle)
        transform[1][1] = cosd(angle)
        return self.transform(transform)

    def __mul__(self, other):
        return Transformation(matrix=np.dot(self.matrix, other.matrix))

    def position(self):
        """ return the positional part of the transformation as a vector """
        beta = math.atan2(self.matrix[2][0], math.sqrt(self.matrix[0][0]**2 + self.matrix[1,0]**2))
        cbeta = math.cos(beta)
        alpha = math.atan2(self.matrix[1][0] / cbeta, self.matrix[0][0] / cbeta)
        gamma = math.atan2(self.matrix[2][1] / cbeta, self.matrix[2][2] / cbeta)
        return [ self.matrix[0][3], self.matrix[1][3], self.matrix[2][3] , math.degrees(alpha), math.degrees(beta), math.degrees(gamma)]

    def __str__(self):
        return str(self.matrix)
    
    def cart_values(self):
        cv = str(self.position()[0]) + ";"
        cv = cv + str(self.position()[1]) + ";"
        cv = cv + str(self.position()[2]) + ";"
        cv = cv + str(self.position()[3]) + ";"
        cv = cv + str(self.position()[4]) + ";"
        cv = cv + str(self.position()[5])
        return cv

class TestDirectKinematics(unittest.TestCase):
    def testNullTransformation(self):
        nullTransformation=JointSpaceVector(angles=[0,0,-90,0,0,0])
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
    unittest.main()
