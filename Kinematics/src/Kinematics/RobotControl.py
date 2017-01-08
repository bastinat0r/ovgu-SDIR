import numpy as np
import math
import Kinematics as kin
from openravepy import *

class RobotControl:
    def __init__(self, robot):
        self.robot = robot
        self.manip = interfaces.BaseManipulation(robot)
    
    def GetTCPTransformation(self):
        return kin.Transformation(matrix=self.robot.GetActiveManipulator().GetLocalToolTransform()) 

    def Move(self, jsv):
        self.manip.MoveActiveJoints(jsv.angles)

    def GetTrajectory(self, jsv, ignoreCollisions=False):
        cc = self.robot.GetEnv().GetCollisionChecker()
        if ignoreCollisions:
            cc = self.robot.GetEnv().SetCollisionChecker(None)
        rval = self.manip.MoveActiveJoints(jsv.angles, outputtrajobj=True, execute=False)
        cc = self.robot.GetEnv().SetCollisionChecker(cc)
        return rval

        rval
    def GetJointValuesFromTrajectory(self, trajectory, time):
        spec = trajectory.GetConfigurationSpecification()
        return spec.ExtractJointValues(trajectory.Sample(time),self.robot,range(self.robot.GetDOF()))

    def _checkCollision(self, jsv):
        """
        inner function for checking robot collisions
        you shoud hide the robot during the process and safe/restore the robot configuration
        """
        self.robot.SetDOFValues(jsv.angles)
        return self.robot.GetEnv().CheckCollision(self.robot)


    def checkRobotCollision(self, jsv):
        """ 
        Check if given Joint-Space-Vector corrosponds to a colliding configuration
        * hide the robot
        * move to given configuration
        * show the robot
        * return False if no collision occurs
        """
        dof = self.robot.GetDOFValues()
        # collision configuration for testing: kin.JointSpaceVector[0, -.5, -.5, 0,0,0]
        self.robot.SetVisible(False)
        rval = self._checkCollision(jsv)
        self.robot.SetDOFValues(dof)
        self.robot.SetVisible(True)
        return rval


