"""Multiple CF: takeoff, follow absolute-coords waypoints, land."""

import numpy as np

from pycrazyswarm import Crazyswarm
from math import *

radius = 0.5
Z = 0.3
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 1.5
r = 3

#The lower this value the higher quality the circle is with more points generated
# divide 2*pi by 12 : 6 positions for each crazyflie but with one step between
stepSize = 0.523598776

#Generated vertices
positions = []

t = 0
while t < 2 * pi:
    positions.append((round(r * cos(t),1), round(r * sin(t),1), Z))
    t += stepSize

# counterclockwise waypoints
WAYPOINTS = np.array(positions)

def nearestPoint(actualPos,positions):
    distances = []
    for p in positions:
        distances.append(dist([p[0],p[1]],[actualPos[0],actualPos[1]]))
    minDistIndexes = np.argsort(distances)

    return minDistIndexes

waypoints_cfs = []
initialPositionIndexes = []

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cfs = allcfs.crazyflies
# add a vector with all the nearest positions index to control that in which point there is no more than 1 crazyflie
    for cf in cfs:
        actualPos = cf.position()
        indexes = nearestPoint(actualPos,WAYPOINTS)
        i = 0
        if indexes[i] in initialPositionIndexes :
            i += 1
            initialPositionIndexes.append(indexes[i])
        else:
            initialPositionIndexes.append(indexes[i])

        waypoints_cf = []
        i = 0
        p = 0
        while p <= 12:
        #return to the initial position
            if i < 11:
               waypoints_cf.append(positions[i])
               i += 1
            else:
                waypoints_cf.append(positions[i])
                i = 0
            p += 1
        waypoints_cfs.append(waypoints_cf)
    
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    for j in range (0,12):
        i=0
        for cf in cfs:
            cf.goTo(waypoints_cfs[i][j], yaw=0.0, duration=GOTO_DURATION)
            i += 1
        timeHelper.sleep(0.1)


    allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


if __name__ == "__main__":
    main()
