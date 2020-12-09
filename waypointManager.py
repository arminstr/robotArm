import os, json, uuid, math
import numpy as np
from dataclasses import dataclass
from robotArm import RobotArm


class WaypointManager(object):
    def __init__(self, uI):
        self.updateInterval = uI
        self.waypoints = []
        self.aWI = 0    #activeWaypointIndex
        self.robotArm = RobotArm(self.updateInterval)
        self.robotArm.updateVisu(self.robotArm.q)
        self.addWaypoint(0.5, 0, 0, 0, 0.1)
    
    def update(self):
        if len(self.waypoints) > 2:
            self.distance = self.getDistanceToIsPositon( self.waypoints[self.aWI] )
            
            if self.distance < ( self.waypoints[self.aWI].lDP * self.waypoints[self.aWI].distanceToNext ):
                self.waypoints[self.aWI].status = 'passed'
                self.aWI += 1
                if self.aWI >= len(self.waypoints):
                    self.aWI = len(self.waypoints) - 1
                    
                self.waypoints[self.aWI].status = 'active'
                self.robotArm.setTargetPosCartesian( self.waypoints[self.aWI].x, self.waypoints[self.aWI].y, self.waypoints[self.aWI].z, self.waypoints[self.aWI].beta)
            #print(self.waypoints[self.aWI], self.aWI, self.distance, self.robotArm.pTarget)    
            self.robotArm.update()
            
    # returns the distance between p1 and self.robotArm.pos
    def getDistanceToIsPositon(self, p1):
        distance = np.linalg.norm(np.array([[ \
            p1.x, p1.y, p1.z \
                ]]).T - self.robotArm.pos)
        return distance
    
    def getDistanceTwoPoints(self, p1, p2):
        distance = np.linalg.norm(np.array([[ p1.x, p1.y, p1.z ]]).T - np.array([[ p2.x, p2.y, p2.z ]]).T)
        return distance

    # lDP = loopDistancePercent
    def addWaypoint(self, x, y, z, beta, lDP):
        id = str(uuid.uuid1())
        waypoint = WayPoint(x, y, z, beta, lDP, 'created', id)
        if len(self.waypoints) > 0:
            waypoint.distanceToNext = self.getDistanceTwoPoints(waypoint, self.waypoints[len(self.waypoints) - 1])
        else:
            waypoint.status = 'active'
            waypoint.distanceToNext = 0.5

        self.waypoints.append(waypoint)    

@dataclass
class WayPoint:
    x: float
    y: float
    z: float
    beta: float
    lDP: float
    status: str
    id: str
    distanceToNext: float = 0.0