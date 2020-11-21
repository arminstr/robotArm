# copyright 2020 Armin Straller
import os, json, uuid, math
import numpy as np
from model import Model
from controller import MotorPositionController

class RobotArm(object):
    def __init__(self, uI):
        self.updateInterval = uI
        #axis limits
        self.limitP = np.array([math.pi * (3/4), math.pi * (3/4), math.pi * (3/4), 100])
        self.limitN = np.array([- math.pi * (3/4), - math.pi * (3/4), - math.pi * (3/4), 0])
        
        #physics model of the robot
        self.robotModel = Model(0.1, self.updateInterval)
        #commanded torque
        self.torqueCommand = np.zeros((4, 1)) #Nm
        #controller
        self.robotMotorController = MotorPositionController(self.updateInterval, self.robotModel.maxOmegaMotors)
        
        #interval for visu update
        self.visuInterval = 50
        self.intervalCounter = 0
        
    def update(self):
        #getting the torque Command from the Controller
        self.torqueCommand = self.robotMotorController.update(self.robotModel.posAxis, self.robotModel.speedMotors)
        self.robotModel.setMotorTorques(self.torqueCommand)

        #updating the robot model
        self.robotModel.update()
        
        #updating the visualization
        if(self.intervalCounter > self.visuInterval):
            self.updateVisu(self.robotModel.posAxis)
            self.intervalCounter = 0
        self.intervalCounter += 1

    #storing current position to json file
    def updateVisu(self, pos):
        if isinstance(pos, np.ndarray):
            pos =  pos.ravel().tolist()
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

    #setter function for Target Axis position using Axis Limits
    def setTargetPosAxis(self, target):
        print(target)
        for i in range(4):
            if (target[i] > self.limitP[i]):
                target[i] = self.limitP[i]
            elif (target[i] < self.limitN[i]):
                target[i] = self.limitN[i]
            else: 
                target[i] = target[i]
        self.robotMotorController.setTargetPos(target)
