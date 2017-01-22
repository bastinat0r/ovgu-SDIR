import numpy as np
import math
import Kinematics as kin
import random
import time
from openravepy import *
from RRT import *
from CollObjects import *
import MotionFunctions

class RobotControl:
    def __init__(self, robot):
        self.robot = robot
        self.manip = interfaces.BaseManipulation(robot)
        with robot.GetEnv():
            self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()
        self.InitRRT()
        self.current_node = self.rrt.root

    
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
        MotionFunctions.setDelta(1)
        t = self.GetTrajectory(jsv)
        MotionFunctions.Move(self.robot, t)        
        time.sleep(0.5)

    def GetTrajectory(self, goal, start=None):
        if start == None:
            start = kin.JointSpaceVector(self.robot.GetDOFValues())
        return MotionFunctions.PTPtoConfiguration(start.angles, goal.angles, 'S')

    def CheckTrajectory(self, trajectory):
        if not self.CheckLimits(kin.JointSpaceVector(trajectory[-1])):
            return True
        # collision configuration for testing: kin.JointSpaceVector[0, -.5, -.5, 0,0,0]
        with self.robot.GetEnv():
            dof = self.robot.GetDOFValues()
            self.robot.SetVisible(False)
            rval = False
            for conf in trajectory:
                if self._checkCollision(kin.JointSpaceVector(conf)):
                    rval = True
                    break
            self.robot.SetDOFValues(dof, xrange(6), checklimits=False)
            self.robot.SetVisible(True)
        return rval
        
    def _checkCollision(self, jsv):
        """
        inner function for checking robot collisions
        you shoud hide the robot during the process and safe/restore the robot configuration
        """
        # todo check limits
        self.robot.SetDOFValues(jsv.angles, xrange(6), checklimits=False)
        return self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()

    def checkRobotCollision(self, jsv):
        """ 
        Check if given Joint-Space-Vector corrosponds to a colliding configuration
        * hide the robot
        * move to given configuration
        * show the robot
        * return False if no collision occurs
        """
        # if the limits are not kept we count it as colliding
        if not self.CheckLimits(jsv):
            return True
        # collision configuration for testing: kin.JointSpaceVector[0, -.5, -.5, 0,0,0]
        with self.robot.GetEnv():
            dof = self.robot.GetDOFValues()
            self.robot.SetVisible(False)
            rval = self._checkCollision(jsv)
            self.robot.SetDOFValues(dof, xrange(6), checklimits=False)
            self.robot.SetVisible(True)
        return rval

    def MoveIsCollisionFree(self, startConfig, endConfig):
        """
        return True if it is possible to move from startConfig to endConfig without a collision
        """
        if not self.CheckLimits(endConfig):
            return False
        t = self.GetTrajectory(endConfig, start=startConfig)
        return not self.CheckTrajectory(t)

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
        """
        return True if the given configuration is within the robot's limits
        """
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
        self.robot.SetDOFValues([0,0,0,0,0,0], xrange(6), checklimits=False)
        self.robot.SetVisible(True)

    def checkConnection(self, nodeA, nodeB):
        return self.MoveIsCollisionFree(kin.JointSpaceVector(nodeA.values),
                                        kin.JointSpaceVector(nodeB.values))

    def InitRRT(self, root=None):
        if(root == None):
            root = self.robot.GetDOFValues()
        self.rrt = RRT(root, self.checkConnection, rc=self)

    def InitGoalTree(self, goal):
        self.goalRRT = RRT(goal.angles, self.checkConnection, rc=self)

    def AddRRTPoint(self, values, rrt=None):
        if rrt==None:
            self.rrt = RRT(root, self.checkConnection, rc=self)
        return rrt.addPoint(values)

    def AddRandomRRTPoint(self, near_current=False, chooseBySumDist=False, chooseByCloseNodeCount=False, rrt=None):
        if rrt == None:
            rrt = self.rrt
        bounds = self.robot.GetDOFLimits()
        if(near_current):
            if not (chooseBySumDist or chooseByCloseNodeCount):
                node = random.choice(rrt.nodes)
            else:
                choice = []
                if chooseBySumDist:
                    nodes = sorted(rrt.nodes, key=lambda x: rrt.sumDist(x))
                    choice = choice + nodes[-5:]
                if chooseByCloseNodeCount:
                    nodes = sorted(rrt.nodes, key=lambda x: rrt.closeNodeCount(x, dist=0.3))
                    choice = choice + nodes[-5:]
                node = random.choice(choice)
            random_point = kin.JointSpaceVector(angles=node.values)
            if len(node.parents) > 0:
                parent_point = kin.JointSpaceVector(angles=node.parents[0].values)
            else:
                parent_point = None
            point = self.GetRandomConfiguration(
                    startConfiguration=random_point, 
                    lastConfiguration=parent_point,
                    num_points=1, 
                    length=0.5, 
                    inertia=0)[0].angles
        else:
            point = None
            while point == None:
                point = [random.uniform(x, y) for x, y in zip(bounds[0], bounds[1])]
                if self.checkRobotCollision(kin.JointSpaceVector(point)):
                    point = None
        p = None
        while p == None:
            p = rrt.addPoint(point)
        return p

    def ShowTree(self, startnode=None, rrt=None):
        """
        should show the current rrt tree
        """
        if rrt == None:
            rrt = self.rrt
        if startnode == None:
            startnode=rrt.root
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

    def ConnectConfigurationToTree(self, goal, growth=10, maxiter=np.Inf, near_current=True, chooseBySumDist=True, chooseByCloseNodeCount=True):
        goalNode = self.rrt.addPoint(goal.angles)
        if goalNode != None:
            return goalNode

        i = 0
        while True:
            # check condition
            if i > maxiter:
                return False
            self.ShowTree()
            # generate 10 new points
            for x in xrange(growth):
                self.AddRandomRRTPoint(near_current=near_current, chooseBySumDist=chooseBySumDist, chooseByCloseNodeCount=chooseByCloseNodeCount)
            # try to add the goal-point
            goalNode = self.rrt.addPoint(goal.angles)
            if goalNode != None:
                return goalNode
            i = i + 1


    def GetWaypoints(self, goal):
        goalNode = self.ConnectConfigurationToTree(goal, maxiter=1)
        self.ShowTree()
        currentNode = goalNode
        waypoints = [currentNode]
        while currentNode.parents != []:
            currentNode = currentNode.parents[0]
            waypoints.append(currentNode)
        waypoints.reverse()

        waypointsToRoot = []
        n = self.current_node
        while n != self.rrt.root:
            waypointsToRoot.append(n)
            n = n.parents[0]
        waypointsToRoot.append(self.rrt.root)
        return waypointsToRoot + waypoints

    def OptimizeWaypoints(self, waypoints):
        i = 0
        while i < len(waypoints) - 1:
            j = i + 2
            maxmove = i+1
            while j < len(waypoints) - 1:
                if self.checkConnection(waypoints[i], waypoints[j]):
                    maxmove = j
                j += 1
            waypoints = waypoints[:i+1] + waypoints[maxmove:]
            i += 1
        return waypoints


    def TreeMove(self, goal, optimize=False):
        waypoints = self.GetWaypoints(goal)
        if optimize:
            waypoints = self.OptimizeWaypoints(waypoints)
        for w in waypoints:
            self.Move(kin.JointSpaceVector(w.values))
            self.current_node = w
            time.sleep(0.2)



    def GetRandomGoal(self):
        bounds = self.robot.GetDOFLimits()
        point = None
        while point == None:
            point = [random.uniform(x, y) for x, y in zip(bounds[0], bounds[1])]
            if self.checkRobotCollision(kin.JointSpaceVector(point)):
                point = None
        return kin.JointSpaceVector(point)

    def ShowGoal(self):
        pass

    def MoveToRoot(self):
        while (self.current_node != self.rrt.root):
            jsv = kin.JointSpaceVector(self.current_node.parents[0].values) 
            self.Move(jsv)
            self.current_node = self.current_node.parents[0]
        jsv = kin.JointSpaceVector(self.current_node.values) 
        self.Move(jsv)


    def GrowTree(self, steps=10, rrt=None):
        if rrt == None:
            rrt = self.rrt
        for i in xrange(steps):
            self.AddRandomRRTPoint(near_current=True, chooseBySumDist=True, chooseByCloseNodeCount=True, rrt=rrt)
        self.ShowTree(rrt=rrt)

    def ConnectTrees(self):
        #find good candidates for connecting the trees
        # for now: brut-force connectivity
        for n2 in self.goalRRT.nodes:
            if self.rrt.addPoint(n2.values) != None:
                print("connecting subtrees")
                self.rrt.addSubtreeNode(n2)
                self.goalTree = None
                self.ShowTree()
                return True
        return False

    def NormalRRT(self, goal):
        self.ConnectConfigurationToTree(goal, near_current=True, chooseBySumDist=True, chooseByCloseNodeCount=True)


    def BiRRT(self, goal):
        self.InitGoalTree(goal)
        while not self.ConnectTrees():
            if len(self.rrt.nodes) > len(self.goalRRT.nodes):
                self.GrowTree(rrt=self.goalRRT)
            else:
                self.GrowTree()

