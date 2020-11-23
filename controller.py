# copyright 2020 Armin Straller
# PID controller for motor position control. 
# TODO: Cascaded angular speed control
import os, json, uuid, math
import numpy as np

class PDController(object):
    def __init__(self, proportionalGain, derivativeGain, limit):
        self.P = proportionalGain
        self.D = derivativeGain
        self.limit = limit
    def update(self, xtarget, x, xdot):
        error = xtarget - x
        commandSignal = error * self.P - xdot * self.D
        for i in range(4):
            if(commandSignal[i] > 0.9 * self.limit[i]):
                commandSignal[i] = 0.9 * self.limit[i]
            if(commandSignal[i] < - 0.9 * self.limit[i]):    
                commandSignal[i] = - 0.9 * self.limit[i]
        return commandSignal