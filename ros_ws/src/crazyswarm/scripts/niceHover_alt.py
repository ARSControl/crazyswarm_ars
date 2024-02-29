#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 0.5
if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    i=0
    #Hexagon [6,5,4,3,2,1] see picture niceHoverImage 
    pos_array = np.array([[-0.5,-0.5,1.0],[-0.7, 0.0, 1.0],[-0.5, 0.6, 1.0],[0.5, 0.6, 1.0],[0.7, 0.0, 1.0],[0.5,-0.5,1.0]])
    for cf in allcfs.crazyflies:
        cf.goTo(pos_array[i], 0, 10.0)
        i += 1
    #go to (position,rotation,time interval)
    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.1, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
