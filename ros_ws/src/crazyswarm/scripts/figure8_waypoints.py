"""Multiple CF: takeoff, follow absolute-coords waypoints, land."""

import numpy as np

from pycrazyswarm import Crazyswarm
from math import *

Z = 0.3
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 1.5


# counterclockwise waypoints, initial positions available in InitialPositionsFigure8.csv

WAYPOINTS = np.array([[(0.5,0.5,0.8),(0.0,1.0,1.0),(-0.5,0.5,1.0),(0.5,-0.5,0.8),(0.0,-1.0,0.5),(-0.5,-0.5,0.5)], 
                      [(0.3,0.8,1.0),(-0.3,0.8,1.0),(-0.3,0.3,1.0),(0.3,-0.8,0.8),(-0.3,-0.8,0.5),(-0.3,-0.3,0.5)], 
                      [(0.0,1.0,1.0),(-0.5,0.5,1.0),(0.3,-0.3,1.0),(0.0,-1.0,0.5),(-0.5,-0.5,0.5),(0.3,0.3,0.5)], 
                      [(-0.3,0.8,1.0),(-0.3,0.3,1.0),(0.5,-0.5,0.8),(-0.3,-0.8,0.5),(-0.3,-0.3,0.5),(0.5,0.5,0.8)], 
                      [(-0.5,0.5,1.0),(0.3,-0.3,1.0),(0.3,-0.8,0.8),(-0.5,-0.5,0.5),(0.3,0.3,0.5),(0.3,0.8,1.0)],
                      [(-0.3,0.3,1.0),(0.5,-0.5,0.8),(0.0,-1.0,0.5),(-0.3,-0.3,0.5),(0.5,0.5,0.8),(0.0,1.0,1.0)], 
                      [(0.3,-0.3,1.0),(0.3,-0.8,0.8),(-0.3,-0.8,0.5),(0.3,0.3,0.5),(0.3,0.8,1.0),(-0.3,0.8,1.0)], 
                      [(0.5,-0.5,0.8),(0.0,-1.0,0.5),(-0.5,-0.5,0.5),(0.5,0.5,0.8),(0.0,1.0,1.0),(-0.5,0.5,1.0)],
                      [(0.3,-0.8,0.8),(-0.3,-0.8,0.5),(-0.3,-0.3,0.5),(0.3,0.8,1.0),(-0.3,0.8,1.0),(-0.3,0.3,1.0)], 
                      [(0.0,-1.0,0.5),(-0.5,-0.5,0.5),(0.3,0.3,0.5),(0.0,1.0,1.0),(-0.5,0.5,1.0),(0.3,-0.3,1.0)],
                      [(-0.3,-0.8,0.5),(-0.3,-0.3,0.5),(0.5,0.5,0.8),(-0.3,0.8,1.0),(-0.3,0.3,1.0),(0.5,-0.5,0.8)], 
                      [(-0.5,-0.5,0.5),(0.3,0.3,0.5),(0.3,0.8,1.0),(-0.5,0.5,1.0),(0.3,-0.3,1.0),(0.3,-0.8,0.8)],
                      [(-0.3,-0.3,0.5),(0.5,0.5,0.8),(0.0,1.0,1.0),(-0.3,0.3,1.0),(0.5,-0.5,0.8),(0.0,-1.0,0.5)],  
                      [(0.3,0.3,0.5),(0.3,0.8,1.0),(-0.3,0.8,1.0),(0.3,-0.3,1.0),(0.3,-0.8,0.8),(-0.3,-0.8,0.5)], 
                      [(0.5,0.5,0.8),(0.0,1.0,1.0),(-0.5,0.5,1.0),(0.5,-0.5,0.8),(0.0,-1.0,0.5),(-0.5,-0.5,0.5)]
])


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cfs = allcfs.crazyflies

    allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    
    for p in WAYPOINTS:
        i = 0
        for cf in cfs:
            cf.goTo(p[i], yaw=0.0, duration=GOTO_DURATION)
            i+=1
        timeHelper.sleep(1.0)

    allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


if __name__ == "__main__":
    main()
