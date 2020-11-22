# copyright 2020 Armin Straller
# PID controller for motor position control. 
# TODO: Cascaded angular speed control
import os, json, uuid, math
import numpy as np

class PDController(object):
    def __init__(self, proportionalGain, derivativeGain):
        self.P = proportionalGain
        self.D = derivativeGain
    def update(self, xtarget, x, xdot):
        error = xtarget - x
        commandSignal = error * self.P - xdot * self.D
        return commandSignal


