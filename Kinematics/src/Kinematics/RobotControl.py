import numpy as np
import math
import Kinematics as kin
import random
from openravepy import *

class RobotControl:
    def __init__(self, robot):
        self.robot = robot
        self.manip = interfaces.BaseManipulation(robot)
        with robot.GetEnv():
            self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()

    
    def GetTCPTransformation(self):
        return kin.Transformation(matrix=self.robot.GetActiveManipulator().GetTransform()) 

    def Move(self, jsv):
        with self.robot.GetEnv():
            self.manip.MoveActiveJoints(jsv.angles)

    def GetTrajectory(self, jsv, ignoreCollisions=False):
        cc = self.robot.GetEnv().GetCollisionChecker()
        if ignoreCollisions:
            cc = self.robot.GetEnv().SetCollisionChecker(None)
        rval = self.manip.MoveActiveJoints(jsv.angles, outputtrajobj=True, execute=False, maxiter=10, maxtries=1)
        cc = self.robot.GetEnv().SetCollisionChecker(cc)
        return rval

    def GetJointValuesFromTrajectory(self, trajectory, time):
        spec = trajectory.GetConfigurationSpecification()
        return spec.ExtractJointValues(trajectory.Sample(time),self.robot,range(self.robot.GetDOF()))

    def _checkCollision(self, jsv):
        """
        inner function for checking robot collisions
        you shoud hide the robot during the process and safe/restore the robot configuration
        """
        # todo check limits
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


    def _MoveIsCollisionFree(self, startConfig, endConfig):
        self.robot.SetDOFValues(startConfig.angles)
        try:
            self.GetTrajectory(endConfig)
            return True
        except planning_error:
            return False
        return True

    def MoveIsCollisionFree(self, startConfig, endConfig):
        """
        return True if it is possible to move from startConfig to endConfig without a collision
        """
        # todo: check limits before more expensive computations
        dof = self.robot.GetDOFValues()
        self.robot.SetVisible(False)
        if self._checkCollision(endConfig):
            rval = False
        else:
            rval = self._MoveIsCollisionFree(startConfig, endConfig)
        self.robot.SetDOFValues(dof)
        self.robot.SetVisible(True)
        return rval

    def GetIKSolutions(self, T, IkFilter=IkFilterOptions.CheckEnvCollisions):
        """
        return IK solutions for point with transformation T
        """
        m = kin.Transformation(matrix=T)
        l = self.ikmodel.manip.FindIKSolutions(T, IkFilter)
        if(len(l) > 0):
            return l
        m.rotate_x(0.0001)
        m.rotate_y(0.0001)
        m.rotate_z(0.0001)
        m.translate([0.0001, 0.0001,0.0001])
        l2 = self.ikmodel.manip.FindIKSolutions(m.matrix, IkFilter)
        return l2

    def GetRandomConfiguration(self, startConfiguration=None, lastConfiguration=None, num_points=5, length=1):
        """
        return a list of jointConfigurations the robot can move to
        if a startConfiguration is given, the collision-free movement 
        length is the manhatten-distance moved in joint space (in rad)
        """
        if startConfiguration == None:
            startConfiguration = kin.JointSpaceVector( self.robot.GetDOFValues() )
        configurations = []
        while len(configurations) < num_points:
            random_offset = [random.uniform(-1, 1) for a in xrange(6)]
            random_offset = [length * x / sum(random_offset) for x in random_offset]
            x = kin.JointSpaceVector(angles = random_offset)
            x.add_angles(startConfiguration.angles)
            #check config
            if self.CheckLimits(x) and self.MoveIsCollisionFree(startConfiguration, x):
                configurations.append(x)

        return configurations
    
    def GetCollidingObjects(self):
        """
        Return a list of Colliding objects
        """
        return [x for x in self.robot.GetEnv().GetBodies() if self.robot.GetEnv().CheckCollision(self.robot, x)]

    def CheckLimits(self, jsv):
        values = [ max > v > min for min, v, max in zip(self.robot.GetDOFLimits()[0], jsv.angles, self.robot.GetDOFLimits()[1])]
        return all(values)
