# copyright 2020 Armin Straller
from robotArm import RobotArm
import random, math, time
import numpy as np

starttime = time.time()
interval = 0.001 #call loop every 10ms

roArm = RobotArm(interval)
roArm.setTargetPosAxis(np.array([0, 0, 0, 0]))

i = 0
while 1:
    roArm.update()
    time.sleep(interval - ((time.time() - starttime) % interval))
    i += 1
    if (i == 5000):
        roArm.setTargetPosAxis(np.array([math.pi*(-2/4), math.pi*(3/4), 0, 0]))
    if (i == 10000):
        roArm.setTargetPosAxis(np.array([math.pi*(2/4), math.pi*(-3/4), 0, 0]))
        i = 0

    #i = i + 0.0005
    #roArm.setTargetPosAxis(np.array([np.sin(i) * math.pi/2, np.sin(i) * math.pi/2, 0, 0]))
    #roArm.update()
    #if(i > 2 * math.pi):
    #    i = 0