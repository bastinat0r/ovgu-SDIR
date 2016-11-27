import numpy as np
import math
import unittest

def cosd(angle):
    """cosine of angle in degree"""
    return math.cos(math.radians(angle))

def sind(angle):
    """sine of angle in degree"""
    return math.sin(math.radians(angle))




def direct_kinematics(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6):
    """ calculate direct kinematics """
    t01 = Transformation(alpha=90,   a=260,  theta=theta_1,     d=-675)
    t12 = Transformation(alpha=0,    a=680,  theta=theta_2,     d=0)
    t23 = Transformation(alpha=90,   a=-35,  theta=theta_3,     d=0)
    t34 = Transformation(alpha=-90,  a=0,    theta=theta_4,     d=-670)
    t45 = Transformation(alpha=90,   a=0,    theta=theta_5,     d=0)
    t5TCP = Transformation(alpha=0,  a=0,    theta=theta_6,     d=-115)
    return t01 * t12 * t23 * t34 * t45 * t5TCP

class JointSpaceVector(object):
    def __init__(self, angles=[0,0,0,0,0,0]):
        self.angles = angles

    def transformation(self):
        return direct_kinematics(self.angles[0], self.angles[1], self.angles[2], self.angles[3], self.angles[4], self.angles[5])
    


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
        return [ self.matrix[0][3], self.matrix[1][3], self.matrix[2][3]  ]

    def __str__(self):
        return str(self.matrix)

class TestDirectKinematics(unittest.TestCase):
    def testNullPosition(self):
        nullPosition=JointSpaceVector(angles=[0,0,-90,0,0,0])
        desired_result = np.array([
            [.0,.0,-1.0,1725.0],
            [.0,-1.0,.0,.0],
            [-1.0,.0,.0,-640.0],
            [.0,.0,.0,1.0]],dtype=np.float64)
        np.testing.assert_allclose(nullPosition.transformation().matrix, desired_result, atol=0.001)

if __name__ == '__main__':
    unittest.main()
