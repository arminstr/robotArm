# copyright 2020 Armin Straller
from robotArm import RobotArm
import random, math, time
import numpy as np

starttime = time.time()
interval = 0.01 #call loop every 10ms

roArm = RobotArm(interval)
roArm.setTargetPosAxis(np.array([0, 0, 0, 0]))

i = 0
while 1:
    roArm.update()
    time.sleep(interval - ((time.time() - starttime) % interval))
    i += 1
    if (i > interval * 10000 * 3):
        roArm.setTargetPosAxis(np.array([-math.pi*(0.5/4), math.pi*(3/4), 0, 0]))
        i = 0
    print(i)