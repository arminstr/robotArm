# copyright 2020 Armin Straller
from robotArm import RobotArm
import random, math, time
import numpy as np

starttime = time.time()
interval = 0.01 #call loop every 10ms

roArm = RobotArm(interval)

i = 0
while 1:
    i = i + 0.01
    roArm.setTargetPosAxis(np.array([np.sin(i) * math.pi/2, -np.sin(i) * math.pi/2, 0, np.sin(i) * 50 + 50]))
    roArm.update()
    #print(i)
    if(i > 2 * math.pi):
        i = 0
    time.sleep(interval - ((time.time() - starttime) % interval))