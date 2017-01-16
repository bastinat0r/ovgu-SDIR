import numpy as np
import math
import Kinematics as kin
import random
import time
from openravepy import *
from RRT import *
from CollObjects import *

class RobotControl:
    def __init__(self, robot):
        self.robot = robot
        self.manip = interfaces.BaseManipulation(robot)
        with robot.GetEnv():
            self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()

    
    def GetTCPTransformation(self, configuration=None):
        if configuration == None:
            return kin.Transformation(matrix=self.robot.GetActiveManipulator().GetTransform()) 

        with self.robot.GetEnv():
            dof = self.robot.GetDOFValues()
            self.robot.SetVisible(False)
            self.robot.SetDOFValues(configuration.angles, xrange(6), checklimits=False)
            rval = kin.Transformation(matrix=self.robot.GetActiveManipulator().GetTransform())
            self.robot.SetDOFValues(dof, xrange(6), checklimits=False)
            self.robot.SetVisible(True)
        return rval

    def Move(self, jsv):
        with self.robot.GetEnv():
            self.manip.MoveActiveJoints(jsv.angles, maxiter=10, maxtries=1, jitter=0)

    def GetTrajectory(self, jsv, ignoreCollisions=False):
        cc = self.robot.GetEnv().GetCollisionChecker()
        if ignoreCollisions:
            cc = self.robot.GetEnv().SetCollisionChecker(None)
        rval = self.manip.MoveActiveJoints(jsv.angles, outputtrajobj=True, execute=False, maxiter=10, maxtries=1, jitter=0)
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
        self.robot.SetDOFValues(jsv.angles, xrange(6), checklimits=False)
        return self.robot.GetEnv().CheckCollision(self.robot)


    def checkRobotCollision(self, jsv):
        """ 
        Check if given Joint-Space-Vector corrosponds to a colliding configuration
        * hide the robot
        * move to given configuration
        * show the robot
        * return False if no collision occurs
        """
        # collision configuration for testing: kin.JointSpaceVector[0, -.5, -.5, 0,0,0]
        with self.robot.GetEnv():
            dof = self.robot.GetDOFValues()
            self.robot.SetVisible(False)
            rval = self._checkCollision(jsv)
            self.robot.SetDOFValues(dof, xrange(6), checklimits=False)
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

    def GetRandomConfiguration(self, startConfiguration=None, lastConfiguration=None, num_points=5, length=1, inertia=0):
        """
        return a list of jointConfigurations the robot can move to
        if a startConfiguration is given, the collision-free movement 
        length is the manhatten-distance moved in joint space (in rad)
        """
        if startConfiguration == None:
            startConfiguration = kin.JointSpaceVector( self.robot.GetDOFValues() )

        if lastConfiguration != None:
            startConfiguration.angles = [x + (x - y) * inertia for x, y in zip(startConfiguration.angles, lastConfiguration.angles)]
        configurations = []
        while len(configurations) < num_points:
            random_offset = [random.uniform(-1, 1) for a in xrange(6)]
            max_v = max([abs(x) for x in random_offset])
            random_offset = [length * x / max_v for x in random_offset]
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

    def RandomWalk(self, steps, sleeptime=1, length=1, inertia=1):
        lastConfiguration = None
        currentConfiguration = None
        for i in xrange(steps):
            newConfiguration = self.GetRandomConfiguration(lastConfiguration=lastConfiguration, num_points=1, length=length, inertia=inertia)[0]
            self.Move(newConfiguration)
            lastConfiguration=currentConfiguration
            currentConfiguration=newConfiguration
            time.sleep(sleeptime)

    def reset(self):
        """
        currently not working :(
        """
        self.robot.SetVisible(False)
        self.robot.SetDOFValues([0,0,0,0,0,0])
        self.robot.SetVisible(True)

    def checkConnection(self, nodeA, nodeB):
        return self.MoveIsCollisionFree(kin.JointSpaceVector(nodeA.values),
                                        kin.JointSpaceVector(nodeB.values))

    def InitRRT(self):
        self.rrt = RRT(self.robot.GetDOFValues(), self.checkConnection)

    def AddRRTPoint(self, values):
        return self.rrt.addPoint(values)

    def AddRandomRRTPoint(self, near_current=False):
        bounds = self.robot.GetDOFLimits()
        if(near_current):
            random_point = kin.JointSpaceVector(angles=random.choice(self.rrt.nodes).values)
            point = self.GetRandomConfiguration(startConfiguration=random_point,num_points=1, length=0.5, inertia=0)[0].angles
        else:
            point = None
            while point == None:
                point = [random.uniform(x, y) for x, y in zip(bounds[0], bounds[1])]
                if self.checkRobotCollision(kin.JointSpaceVector(point)):
                    point = None
        p = None
        while p == None:
            p = self.rrt.addPoint(point)
        return p

    def ShowTree(self, startnode=None):
        """
        should show the current rrt tree
        """
        if startnode == None:
            startnode=self.rrt.root
        self.handles = []
        self.ShowNode(startnode)

    def ShowNode(self, node, depth=0):
        d = depth + 1
        node_pos = self.GetTCPTransformation(configuration=kin.JointSpaceVector(node.values)).position()
        node_pos = list(node_pos[0:3])
        for child in node.children:
            child_pos = self.GetTCPTransformation(configuration=kin.JointSpaceVector(child.values)).position()
            child_pos = list(child_pos[0:3])
            self.handles.append(drawExample(self.robot.GetEnv(), node_pos + child_pos, presskey=False, depth=d))
            self.ShowNode(child, depth=d)
        


