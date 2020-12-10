# copyright 2020 Armin Straller
from waypointManager import WaypointManager
import random, math, time
import numpy as np



starttime = time.time()
interval = 0.001 #call loop every 1ms

waypontM = WaypointManager(interval)

waypontM.addWaypoint('PTP',     0.1,    0,      0,      0)
waypontM.addWaypoint('PTP',     0.1,    0,      0.2,    0)
waypontM.addWaypoint('PTP',     0.1,    0,      0,      0, 0.5)
waypontM.addWaypoint('PTP',     0.2,    -0.2,   0,      0)
waypontM.addWaypoint('PTP',     0.2,    -0.2,   0.2,    0)
waypontM.addWaypoint('PTP',     0.2,    -0.2,   0,      0, 0.5)
waypontM.addWaypoint('PTP',     0.2,    0.2,    0,      0, 0.5)
waypontM.addWaypoint('PTP',     -0.1,   0.2,    0,      0)
waypontM.addWaypoint('PTP',     -0.1,   0.2,    0.2,    0)
waypontM.addWaypoint('PTP',     -0.1,   0.2,    0,      0, 0.5)
waypontM.addWaypoint('PTP',     0.2,    0.2,    0,      0, 0.5)
waypontM.addWaypoint('PTP',     0.5,    0,      0,      0, 0.1)

time.sleep(3)
while 1:
    waypontM.update()
    time.sleep(interval - ((time.time() - starttime) % interval))
        