import numpy as np
import Kinematics as kin
from RobotControl import *
import math

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
        return node

    def addSubtreeNode(self, otherNode):
        own = self.addPoint(otherNode.values)
        if own == None:
            return

        parent = own.parents[0]
        self.nodes.remove(own)
        current = otherNode
        lastNode = None
        while len(current.parents) > 0:
            # * put old parent in list of childs
            # * add new parent and remove from list of childs
            oldParent = current.parents[0]
            current.parents = [parent]
            if parent in current.children:
                current.children.remove(parent)
            #add all children (and children of children) to the current tree
            self._addSubtreeNode(current)
            #add old parent to list of children
            current.children.append(oldParent)
            parent.children.append(current)
            parent = current
            #go on with next in list
            current = oldParent

        print(parent)
        print(current)
        print([str(x) for x in current.children])
        # also add the root node
        if parent in current.children:
            print(" remove old parent ")
            current.children.remove(parent)
        current.parents = [parent]
        self._addSubtreeNode(current)

        
    def _addSubtreeNode(self, subtreeNode):
        """
        recursive function to add child nodes to own node list
        """
        self.nodes.append(subtreeNode)
        for n in subtreeNode.children:
            self.nodes.append(n)
            print(".")
            self._addSubtreeNode(n)

    def sumDist(self, node):
        return sum([node.cart_distance(n) for n in self.nodes])

    def closeNodeCount(self, node, dist=1):
        closeNodes = [n for n in self.nodes if node.cart_distance(n) < dist]
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
        return math.sqrt(sum([(x - y)**2 for x, y in zip(self.values, other.values)]))

    def cart_distance(self, other):
        """
        distance measure for the nodes in this rrt
        """
        return math.sqrt(sum([(x - y)**2 for x, y in zip(self.point, other.point)]))
    
    def __str__(self):
        return "%s" %str(self.values)
