import os, json, uuid, math
import numpy as np
from dataclasses import dataclass
from robotArm import RobotArm


class WaypointManager(object):
    def __init__(self, uI):
        self.updateInterval = uI
        self.waypoints = []
        self.aWI = 0    #activeWaypointIndex
        self.LINNumDistance = 0.01 # 1cm
        self.robotArm = RobotArm(self.updateInterval)
        self.robotArm.updateVisu(self.robotArm.q)
        self.addWaypoint('PTP', 0.5, 0, 0, 0, 0.1)
    
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
    def addWaypoint(self, type, x, y, z, beta, lDP = 0.01):
        id = str(uuid.uuid1())
        waypoint = WayPoint(type, x, y, z, beta, lDP, 'created', id)

        #PTP Mode just directly sets the Waypoint as the Target Position ( with tolerance according to lDP)
        if type == 'PTP':
            if len(self.waypoints) > 0:
                waypoint.distanceToNext = self.getDistanceTwoPoints(waypoint, self.waypoints[len(self.waypoints) - 1])
            else:
                waypoint.status = 'active'
                waypoint.distanceToNext = 0.5
            self.waypoints.append(waypoint)
        
        #LIN Mode will intrapolate other Waypoints every cm between the last waypoint and the target waypoint
        if type == 'LIN':
            if len(self.waypoints) > 0:
                waypoint.distanceToNext = self.getDistanceTwoPoints(waypoint, self.waypoints[len(self.waypoints) - 1])

                # find out the linear equation between start and end point
                gradient = np.zeros((3,1))
                gradient[0] = waypoint.x / self.waypoints[self.aWI].x
                gradient[1] = waypoint.y / self.waypoints[self.aWI].y
                gradient[2] = waypoint.z / self.waypoints[self.aWI].z

                offset = np.zeros((3,1))
                gradient[0] = waypoint.x / self.waypoints[self.aWI].x
                gradient[1] = waypoint.y / self.waypoints[self.aWI].y
                gradient[2] = waypoint.z / self.waypoints[self.aWI].z

                
                for i in range(numIntrapolations):





                self.waypoints.append(waypoint)
            else:
                raise ValueError('A LIN type Waypoint can not be the first Waypoint!')
            


@dataclass
class WayPoint:
    type: str
    x: float
    y: float
    z: float
    beta: float
    lDP: float
    status: str
    id: str
    distanceToNext: float = 0.0