# copyright 2020 Armin Straller
from robotArm import RobotArm
import random, math, time
import numpy as np

starttime = time.time()
interval = 0.001 #call loop every 1ms

roArm = RobotArm(interval)
roArm.updateVisu(roArm.q)
time.sleep(5)
i = 0
while 1:
    roArm.update()
    time.sleep(interval - ((time.time() - starttime) % interval))
    i += 1
    if (i == 5000):
        roArm.setTargetPosCartesian(0.2, 0, 0.2, 0)
        #targetPos = np.array([[math.pi*(2/4), - math.pi*(2/4),0.2, 2 * math.pi]]).T
        #roArm.setTargetPosAxis(targetPos)
    if (i == 6000):
        roArm.setTargetPosCartesian(0.4, 0.3, 0.1, 0)
        #targetPos = np.array([[-math.pi*(2/4), + math.pi*(2/4),0.2, 2 * math.pi]]).T
        #roArm.setTargetPosAxis(targetPos)
    if (i == 7000):
        roArm.setTargetPosCartesian(0.5, 0, 0, 0)
        #targetPos = np.array([[-math.pi*(2/4), + math.pi*(2/4),0.2, 2 * math.pi]]).T
        #roArm.setTargetPosAxis(targetPos)

        i = 0

    #i = i + 0.0005
    #Z axis position in meters
    #roArm.setTargetPosAxis(np.sin(i) * math.pi/2, - np.cos(i) * math.pi/2, i * 0.025, 4 * np.sin(i) * math.pi)
    #roArm.update()
    #if(i > 2 * math.pi):
    #    i = 0