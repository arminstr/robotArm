# copyright 2020 Armin Straller
# PID controller for motor position control. 
# TODO: Cascaded angular speed control
import os, json, uuid, math
import numpy as np

class MotorPositionController(object):
    def __init__(self, uI, maxMotorSpeed):
        self.updateInterval = uI
        self.targetMotorPos = np.zeros((4, 1))
        self.targetMotorSpeed = np.array([0, 0, 0, 0])
        self.targetMotorTorque = np.zeros((4, 1))
        self.posMotor = np.zeros((4, 1))
        self.speedMotor = np.zeros((4, 1))
        self.maxMotorSpeed = maxMotorSpeed

        self.KpP = np.array([2.3, 4.1, 0, 0]) #6.84 #8.7
        self.KiP = np.array([0.0005, 0.0007, 0, 0])
        self.KdP = np.array([350, 0, 0, 0]) #600 #1250
        self.integralP = np.zeros((4, 1))
        self.lastErrorP = np.zeros((4, 1))
        self.pClamp = 0
        self.pSignComp = 0

        self.KpV = np.array([1, 1, 0.2, 0])
        self.KiV = np.array([0.01, 0.01, 0, 0])
        self.integralV = np.zeros((4, 1))
        

    def update(self, posAxis, speedMotors):
        self.posMotor = posAxis
        self.speedMotor = speedMotors
        for i in range(4):
            self.positionController(i)
            self.speedController(i)
        print(self.targetMotorPos[1],self.posMotor[1], self.integralP[1], self.targetMotorSpeed[1], self.speedMotor[1])
        return self.targetMotorTorque

    def speedController(self, id):
        error = self.targetMotorSpeed[id] - self.speedMotor[id]
        self.integralV[id] += error * self.KiV[id]
        self.targetMotorTorque[id] = error * self.KpV[id] + self.integralV[id]

    def positionController(self, id):
        #error calculation
        error = self.targetMotorPos[id] - self.posMotor[id]
        
        #check if the output is clamped and the error sign is different from the actuator command value sign
        if(self.pClamp and self.pSignComp):
            pass
        else:
            self.integralP[id]  += error * self.KiP[id]
        
        #derivative part of the pid
        derivate = error - self.lastErrorP[id]  
        self.lastErrorP[id] = error

        commandValue = error * self.KpP[id] + self.integralP[id] + derivate * self.KdP[id]
        #commandValue clamping
        self.pClamp = 0
        if(commandValue > 0.9 * self.maxMotorSpeed[id]):
            self.pClamp = 1
            commandValue = 0.9 * self.maxMotorSpeed[id]
        if(commandValue < 0.9 * - self.maxMotorSpeed[id]):
            self.pClamp = 1
            commandValue = 0.9 * - self.maxMotorSpeed[id]
        
        #sign comparison
        self.pSignComp = 0
        if (error < 0 and commandValue < 0 or error > 0 and commandValue > 0):
            self.pSignComp = 1
        
        self.targetMotorSpeed[id] = commandValue

    # send limited target Position Array 
    def setTargetPos(self, targetPos):
        self.targetMotorPos = targetPos