# copyright 2020 Armin Straller
import os, json, uuid, math
import numpy as np

class AxisController(object):
    def __init__(self, uI):
        self.updateInterval = uI
        self.targetPos = np.zeros((4, 1))

    def update(self):
        motorTorques = np.zeros((4, 1)) #Nm

        return motorTorques

    def speedController(self, id):
        pass

    def positionController(self, id):
        pass

    def setTarget(self, targetPos):
        pass