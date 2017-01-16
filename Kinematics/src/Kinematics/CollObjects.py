from openravepy import *
import random
import numpy as np

def drawExample(env, values, presskey=True, depth=0):
    values = [float(x) for x in values]
    x1 = float(values[0])
    y1 = float(values[1])
    z1 = float(values[2])
    x2 = float(values[3])
    y2 = float(values[4])
    z2 = float(values[5])
    handles = []
    handles.append(env.drawlinestrip(points=np.array(((x1, y1, z1),(x2, y2, z2))), linewidth=0.01, colors=np.array(((255-depth)%255, (depth+128)%255, (255-depth) % 255))))
    if(presskey):
        raw_input('Enter any key to quit. ')
    return handles[0]
    
    
class CreateObject():
    objects = []
    limits = {
                'x': (-2, 2),
                'y': (-2, 2),
                'z': (0, 4),
                'b_x': (0, 1),
                'b_y': (0, 1),
                'b_z': (0, 1)
             }
    
    def __init__(self, env, values = None, name = None):    
        if name == None:
            name = 'obj' + str(len(self.objects)) 
        if values == None:
            values = numpy.array([[random.uniform(self.limits['x'][0],self.limits['x'][1]), 
                                   random.uniform(self.limits['y'][0],self.limits['y'][1]), 
                                   random.uniform(self.limits['z'][0],self.limits['z'][1]), 
                                   random.uniform(self.limits['b_x'][0],self.limits['b_x'][1]), 
                                   random.uniform(self.limits['b_y'][0],self.limits['b_y'][1]), 
                                   random.uniform(self.limits['b_z'][0],self.limits['b_z'][1])]])
        
        self.body = RaveCreateKinBody(env,'')
        self.body.SetName(name)
        self.body.InitFromBoxes(values,True)
        
        self.objects.append(self.body)
        env.Add(self.objects[-1],True)
