import numpy as np
import Kinematics as kin
from RobotControl import *

class RRT:
    def __init__(self, root_node, connects, rc=None):
        """
        values for root node, function to test if two vertices may be connected
        """
        self.rc = rc
        self.root = RRTNode(root_node, rc=self.rc)
        self.nodes = [self.root]
        self.connects = connects

    def addPoint(self, point):
        #find closest
        node = RRTNode(point, rc=self.rc)
        neighbour = None
        dist = np.inf
        sorted_nodes = sorted(self.nodes, key=lambda x: node.distance(x))
        #check three nearest nodes to find a connectable one
        for n in sorted_nodes[:3]:
            if self.connects(n, node):
                neighbour = n
                break

        #add node to parent
        if neighbour == None:
            return None
        neighbour.children.append(node)
        node.parents.append(neighbour)
        self.nodes.append(node)
        return neighbour

    def sumDist(self, node):
        return sum([node.distance(n) for n in self.nodes])

    def closeNodeCount(self, node, dist=1):
        closeNodes = [n for n in self.nodes if node.distance(n) < dist]
        return len(closeNodes)

    
class RRTNode:
    def __init__(self, values, rc=None):
        self.children = []
        self.parents = []
        self.values = values
        if rc != None:
            configuration = kin.JointSpaceVector(angles=values)
            self.point = rc.GetTCPTransformation(configuration=configuration).position()[:3]


    def distance(self, other):
        """
        distance measure for the nodes in this rrt
        """
        return sum([(x - y)**2 for x, y in zip(self.point, other.point)])
    
    def __str__(self):
        return "%s" %str(self.values)
