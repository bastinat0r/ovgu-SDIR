import numpy as np
import math

def cosd(angle):
    """cosine of angle in degree"""
    return math.cos(math.radians(angle))

def sind(angle):
    """sine of angle in degree"""
    return math.sin(math.radians(angle))



def direct_kinematics(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6):
    """ calculate direct kinematics """
    t01 = Transformation(alpha=90,   a=260,  theta=theta_1,     d=-675)
    t12 = Transformation(alpha=0,    a=680,  theta=theta_2-90,  d=0)
    t23 = Transformation(alpha=90,   a=-35,  theta=theta_3,     d=0)
    t34 = Transformation(alpha=-90,  a=0,    theta=theta_4,     d=-670)
    t45 = Transformation(alpha=90,   a=0,    theta=theta_5,     d=0)
    t5TCP = Transformation(alpha=0,  a=0,    theta=theta_6,     d=-115)


    return t01 * t12 * t23 * t34 * t45 * t5TCP




class Transformation(object):


    """A simple class wrapping a standart robot transformation"""
    def __init__(self, matrix=np.eye(4,4), a=0, d=0, alpha=0, theta=0):
        """Introduce Epmty Transformation"""
        self.matrix = matrix
        self.rotate_z(alpha)
        self.translate([0, 0, a])
        self.translate([d, 0, 0])
        self.rotate_x(theta)
        pass

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
        """ transform self by given matrix """
        self.matrix = np.dot(matrix, self.matrix)
        return self

    def translate(self, vector3):
        """ translate matrix by given vector3 """
        transform = np.eye(4, 4)
        transform[0][3] = vector3[0]
        transform[1][3] = vector3[1]
        transform[2][3] = vector3[2]
        return self.transform(transform)


    def rotate_x(self, angle):
        """ rotate around the x axis """
        transform = np.eye(4, 4)
        transform[1][1] = cosd(angle)
        transform[1][2] = -1 * sind(angle)
        transform[2][1] = sind(angle)
        transform[2][2] = cosd(angle)
        return self.transform(transform)


    def rotate_y(self, angle):
        """ rotate around the y axis """
        transform = np.eye(4, 4)
        transform[0][0] = cosd(angle)
        transform[0][2] = -1 * sind(angle)
        transform[2][0] = sind(angle)
        transform[2][2] = cosd(angle)
        return self.transform(transform)


    def rotate_z(self, angle):
        """ rotate around the z axis """
        transform = np.eye(4, 4)
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

if __name__ == '__main__':
    x = direct_kinematics(0,0,-90,0,0,0)
    print(str(x))
    t01 = Transformation(alpha=90,   a=260,  theta=0,     d=-675)
    print(t01)
