import numpy as np

class Transformation(object):
    self.matrix # the 4x4 matrix representing the transformation


    """A simple class wrapping a standart robot transformation"""
    def __init__(self, matrix=np.eye(4,4)):
        """Introduce Epmty Transformation"""
        self.matrix = matrix
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
        self.matrix = np.dot(self.matrix, matrix)
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
        transform[1][1] = cos(angle)
        transform[1][2] = -1 * sin(angle)
        transform[2][1] = sin(angle)
        transform[2][2] = cos(angle)
        return self.transform(transform)


    def rotate_y(self, angle):
        """ rotate around the y axis """
        transform = np.eye(4, 4)
        transform[0][0] = cos(angle)
        transform[0][2] = -1 * sin(angle)
        transform[2][0] = sin(angle)
        transform[2][2] = cos(angle)
        return self.transform(transform)


    def rotate_z(self, angle):
        """ rotate around the z axis """
        transform = np.eye(4, 4)
        transform[0][0] = cos(angle)
        transform[0][1] = -1 * sin(angle)
        transform[1][0] = sin(angle)
        transform[1][1] = cos(angle)
        return self.transform(transform)

