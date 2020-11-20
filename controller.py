# copyright 2020 Armin Straller
# PID controller for axis position control. 
# TODO: Cascaded angular speed control
import os, json, uuid, math
import numpy as np

class AxisController(object, ):
    def __init__(self, uI):
        self.updateInterval = uI
        self.targetAxisPos = np.zeros((4, 1))
        self.targetMotorSpeed = np.array([10, 1, 0, 0])
        self.targetMotorTorque = np.zeros((4, 1))
        self.posAxis = np.zeros((4, 1))
        self.speedMotor = np.zeros((4, 1))

        self.KpP = np.array([1.1, 4, 0, 0])
        self.KiP = np.array([0.0029, 0, 0, 0])
        self.KdP = np.array([1, 0, 0, 0])
        self.integralP = np.zeros((4, 1))
        self.lastErrorP = np.zeros((4, 1))

        self.KpV = np.array([0.2, 0.2, 0.2, 0])
        self.KiV = np.array([0.002, 0.002, 0, 0])
        self.integralV = np.zeros((4, 1))
        

    def update(self, posAxis, speedMotors):
        self.posAxis = posAxis
        self.speedMotor = speedMotors
        for i in range(4):
            self.positionController(i)
            self.speedController(i)
        print(self.targetAxisPos[0],self.posAxis[0], self.integralP[0], self.speedMotor[0])
        return self.targetMotorTorque

    def speedController(self, id):
        error = self.targetMotorSpeed[id] - self.speedMotor[id]
        self.integralV[id] += error
        self.targetMotorTorque[id] = error * self.KpV[id] + self.integralV[id] * self.KiV[id]

    def positionController(self, id):
        error = self.targetAxisPos[id] - self.posAxis[id]
        self.integralP[id]  += error
        derivate = error - self.lastErrorP[id]
        self.lastErrorP[id] = error
        self.targetMotorSpeed[id] = error * self.KpP[id] + self.integralP[id] * self.KiP[id]  + derivate * self.KdP[id]

    # send limited target Position Array 
    def setTargetPos(self, targetPos):
        self.targetAxisPos = targetPos