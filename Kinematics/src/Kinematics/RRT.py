import numpy as np

class RRT:
    def __init__(self, root_node, connects):
        """
        values for root node, function to test if two vertices may be connected
        """
        self.root = RRTNode(root_node)
        self.nodes = [self.root]
        self.connects = connects

    def addPoint(self, point):
        #find closest
        node = RRTNode(point)
        neighbour = None
        dist = np.inf
        for n in self.nodes:
            if n.distance(node) < dist and self.connects(n, node):
                dist = n.distance(node)
                neighbour = n
        #add node to parent
        if neighbour == None:
            return None
        neighbour.children.append(node)
        self.nodes.append(node)
        return neighbour



    
class RRTNode:
    def __init__(self, values):
        self.children = []
        self.values = values


    def distance(self, other):
        """
        distance measure for the nodes in this rrt
        """
        return max([abs(x - y) for x, y in zip(self.values, other.values)])
    
    def __str__(self):
        return "%s" %str(self.values)
