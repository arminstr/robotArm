# copyright 2020 Armin Straller
from robotArm import RobotArm
import random, math, time
import numpy as np

starttime = time.time()
interval = 0.001 #call loop every 10ms

roArm = RobotArm(interval)
roArm.updateVisu(roArm.q)
time.sleep(5)
i = 0
while 1:
    roArm.update()
    time.sleep(interval - ((time.time() - starttime) % interval))
    i += 1
    if (i == 5000):
        roArm.setTargetPosAxis(math.pi*(2/4), - math.pi*(2/4),0.2, 2 * math.pi)
    if (i == 10000):
        roArm.setTargetPosAxis(-math.pi*(2/4), + math.pi*(3/4),0, -4 * math.pi)
        i = 0

    #i = i + 0.0005
    #Z axis position in meters
    #roArm.setTargetPosAxis(np.sin(i) * math.pi/2, - np.cos(i) * math.pi/2, i * 0.025, 4 * np.sin(i) * math.pi)
    #roArm.update()
    #if(i > 2 * math.pi):
    #    i = 0