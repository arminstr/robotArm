import os, json, uuid, math
import numpy as np
from model import model

class robotArm(object):
    def __init__(self):
        self.limitP = np.array([math.pi * (3/4), math.pi * (3/4), math.pi * (3/4), 100])
        self.limitN = np.array([- math.pi * (3/4), - math.pi * (3/4), - math.pi * (3/4), 0])
        self.targetPos = np.zeros((4, 1))
        self.pos = np.zeros((4, 1))
    def update(self):
        self.updateVisu(self.pos)

    def updateVisu(self, pos):
        filename = 'control.json'
        with open(filename, 'r') as f:
            data = json.load(f)
            data["robotPosition"]["a1"] = pos[0] 
            data["robotPosition"]["a2"] = pos[1]
            data["robotPosition"]["a3"] = pos[2]
            data["robotPosition"]["z"] = pos[3]
        # create randomly named temporary file to avoid 
        # interference with other thread/asynchronous request
        tempfile = os.path.join(os.path.dirname(filename), str(uuid.uuid4()))
        with open(tempfile, 'w') as f:
            json.dump(data, f, indent=4)
        # rename temporary file replacing old file
        os.rename(tempfile, filename)

    def setTargetPosAxis(self, target):
        self.targetPos = target
        self.pos = self.targetPos