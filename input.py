from robotArm import robotArm
import random, math, time
import numpy as np

roArm = robotArm()

i = 0
while 1:
    i = i + 0.1
    roArm.setTargetPosAxis(np.array([np.sin(i) * math.pi/2, -np.sin(i) * math.pi/2, 0, np.sin(i) * 50 + 50]))
    roArm.update()
    time.sleep(0.05)
    if(i > 2 * math.pi):
        i = 0