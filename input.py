# copyright 2020 Armin Straller
from waypointManager import WaypointManager
import random, math, time
import numpy as np



starttime = time.time()
interval = 0.001 #call loop every 1ms

waypontM = WaypointManager(interval)

waypontM.addWaypoint(0.2, 0, 0, 0, 1e-2)
waypontM.addWaypoint(0.2, 0, 0.2, 0, 1e-2)
waypontM.addWaypoint(0.2, 0, 0, 0, 0.5)
waypontM.addWaypoint(0.5, 0, 0, 0, 0.1)
waypontM.addWaypoint(0.2, -0.1, 0, 1 * math.pi, 1e-2)
waypontM.addWaypoint(0.2, -0.1, 0.2, 1 * math.pi, 1e-2)
waypontM.addWaypoint(0.2, -0.1, 0, 1 * math.pi, 0.5)
waypontM.addWaypoint(0.2, 0.2,  0, -1 * math.pi, 1e-2)
waypontM.addWaypoint(0.2, 0.2,  0.2, -1 * math.pi, 1e-2)
waypontM.addWaypoint(0.2, 0.2,  0, -1 * math.pi, 0.5)
waypontM.addWaypoint(0.5, 0, 0, 0, 0.1)

time.sleep(3)
while 1:
    waypontM.update()
    time.sleep(interval - ((time.time() - starttime) % interval))
        