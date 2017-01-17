from openravepy import *
import random
import numpy as np
import json

def drawExample(env, values, presskey=True, depth=0):
    values = [float(x) for x in values]
    x1 = float(values[0])
    y1 = float(values[1])
    z1 = float(values[2])
    x2 = float(values[3])
    y2 = float(values[4])
    z2 = float(values[5])
    handles = []
    r = ((depth * 13) % 127) / 127.0
    g = 1.0 / (depth + 0.001)
    b = ((depth * 3) % 127) / 127.0
    handles.append(env.drawlinestrip(points=np.array(((x1, y1, z1),(x2, y2, z2))), linewidth=2, colors=np.array(((r,g,b)))))
    if(presskey):
        raw_input('Enter any key to quit. ')
    return handles[0]

def saveConfig(env, slot):    
    objects = Obstacles().getGeometrics()
    try:
        with open('testsamples.txt','r') as f:
            strFile = json.load(f)
        strFile[slot] = objects
        
        with open('testsamples.txt','w') as f:
            json.dump(strFile, f)
        
    except:
        createFile()
        saveConfig(env,slot)
            
def loadConfig(env, slot):
    try:
        with open('testsamples.txt','r') as f:
            strFile = json.load(f)
            
        config = strFile[slot]
        obst = Obstacles()
        obst.deleteAllObstacles(env)
        for val in config:
            obst.createObstacles(env, values = val)
    except:
        createFile()
        loadConfig(env,slot)
        
def createFile():
    f = open('testsamples.txt', 'w')
    str_json = {
                   0: [],
                   1: [],
                   2: [],
                   3: [],
                   4: []
                }
    json.dump(str_json, f)
    f.close()
    
    
class Obstacles():
    objects = []
    geometrics = []
    limits = {
                'x': (-2000.0, 2000.0),
                'y': (-2000.0, 2000.0),
                'z': (0.0, 4000.0),
                'b_x': (0.0, 1000.0),
                'b_y': (0.0, 1000.0),
                'b_z': (0.0, 1000.0)
             }
    
    def __init__(self): 
        True
    
    def createObstacles(self, env, values = None, name = None):
        if name == None:
            name = 'obj' + str(len(self.objects)) 
        if values == None:
            values = [random.uniform(self.limits['x'][0],self.limits['x'][1]), 
                      random.uniform(self.limits['y'][0],self.limits['y'][1]), 
                      random.uniform(self.limits['z'][0],self.limits['z'][1]), 
                      random.uniform(self.limits['b_x'][0],self.limits['b_x'][1]), 
                      random.uniform(self.limits['b_y'][0],self.limits['b_y'][1]), 
                      random.uniform(self.limits['b_z'][0],self.limits['b_z'][1])]
        else:
            values = [float(x) for x in values]
        
        self.geometrics.append(values)    
        values = np.divide(values, 1000.0)
        values_arr = numpy.array([values])
        
        self.body = RaveCreateKinBody(env,'')
        self.body.SetName(name)
        self.body.InitFromBoxes(values_arr,True)
        
        self.objects.append(self.body)
        env.Add(self.objects[-1],True)
        
    def deleteAllObstacles(self, env):
        obj = env.GetBodies()[3:]
        for o in obj:
            env.Remove(o)
        self.geometrics = []
        
    def getGeometrics(self):
        return self.geometrics
