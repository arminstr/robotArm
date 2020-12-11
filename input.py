# copyright 2020 Armin Straller
from waypointManager import WaypointManager
import random, math, time
import numpy as np



starttime = time.time()
interval = 0.001 #call loop every 1ms

waypontM = WaypointManager(interval)

waypontM.addWaypoint('PTP',     0.1,    0,      0,      0)
waypontM.addWaypoint('PTP',     0.3,    0.3,    0,      0)
waypontM.addWaypoint('LIN',     0.3,    0.1,    0,      0, 20)
waypontM.addWaypoint('PTP',     0.35,   0.3,    0,    math.pi/2)
waypontM.addWaypoint('LIN',     0.35,   0.1,    0.2,    math.pi/2, 15)
waypontM.addWaypoint('PTP',     0.35,   0.35,   0.1,    math.pi/4)
waypontM.addWaypoint('LIN',     0.1,    0.1,    0.1,    math.pi/4, 10)
waypontM.addWaypoint('PTP',     0.5,    0,      0,      0)

time.sleep(3)
while 1:
    waypontM.update()
    time.sleep(interval - ((time.time() - starttime) % interval))
        