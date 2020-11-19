# copyright 2020 Armin Straller
import math
import numpy as np

class Model(object):
    def __init__(self, attachedLoad, uI):
        self.posAxis = np.zeros((4, 1))
        self.speedMotors = np.zeros((4, 1))
        self.torqueMotors = np.zeros((4, 1))
        self.alphaMotors = np.zeros((3, 1))
        self.omegaMotors = np.zeros((3, 1))
        self.inertiaAxis = np.zeros((3, 1))
        self.accelZ = 0
        self.speedZ = 0
        self.maxOmegaMotors = np.array([20, 20, 20, 20]) #rotations per second
        self.maxTorqueMotors = np.array([1.9, 1.9, 0.5, 0.5]) #Nm
        self.transmissionMotors = np.array([0.5, 0.5, 0.5, 0.125]) #Nm
        self.forceZ = 1.0 #N
        self.maxSpeedZ = 0.01 #m/s
        self.armLength = np.array([0.25, 0.25, 0.01, 0.20]) #meters
        self.load = attachedLoad
        self.armLoads =  np.array([2, 3, self.load]) #kg
        self.updateInterval = uI    

    def update(self):
        self.calculateInertia()
        for i in range(3):
            self.motorController(i)
        self.linearController(3)
    
    # Inertia calculation assumes that the loads of each individual arm act as point load at the center of the arm. 
    # This way the intertia can be easily calculated per simulation cycle depending on the joint angles.
    def calculateInertia(self):
        self.inertiaAxis[2] = (self.armLength[2] / 2) ** 2 * self.armLoads[2]
        self.inertiaAxis[1] = (self.armLength[1] / 2) ** 2 * (self.armLoads[1] + self.armLength[1] ** 2 * self.armLoads[2])
        
        lengthLoad1 = math.sqrt(    (math.cos(self.posAxis[0]) * self.armLength[0] + math.cos(self.posAxis[1]) * (self.armLength[1]/2))**2 + 
                                    (math.sin(self.posAxis[0]) * self.armLength[0] + math.sin(self.posAxis[1]) * (self.armLength[1]/2))**2 )
        lengthLoad2 = math.sqrt(    (math.cos(self.posAxis[0]) * self.armLength[0] + math.cos(self.posAxis[1]) * (self.armLength[1]))**2 + 
                                    (math.cos(self.posAxis[0]) * self.armLength[0] + math.cos(self.posAxis[1]) * (self.armLength[1]))**2 )
        
        self.inertiaAxis[0] = (self.armLength[0] / 2) ** 2 * self.armLoads[0] + lengthLoad1 ** 2 * self.armLoads[1] + lengthLoad2 ** 2 * self.armLoads[2]

    def motorController(self, id):
        self.alphaMotors[id] = (self.torqueMotors[id] / self.transmissionMotors[id]) / self.inertiaAxis[id] # rad / (s**2)
        self.omegaMotors[id] = self.omegaMotors[id] + self.alphaMotors[id] * self.updateInterval
        #limiting speed to maximum Motor angular speed
        if(self.omegaMotors[id] > self.maxOmegaMotors[id]):
            self.omegaMotors[id] = self.maxOmegaMotors[id]
        if(self.omegaMotors[id] < -self.maxOmegaMotors[id]):
            self.omegaMotors[id] = -self.maxOmegaMotors[id]
        self.posAxis[id] = self.posAxis[id] + (self.omegaMotors[id] * self.updateInterval) * self.transmissionMotors[id] # adjust for transmission
        #limiting position to maximum Motor position


    def linearController(self, id):
        pass

    def setMotorTorques(self, torques):
        for i in range(4):
            if (torques[i] > self.maxTorqueMotors[i]):
                self.torqueMotors[i] = self.maxTorqueMotors[i]
            elif (torques[i] < - self.maxTorqueMotors[i]):
                self.torqueMotors[i] = - self.maxTorqueMotors[i]
            else: 
                self.torqueMotors[i] = torques[i]