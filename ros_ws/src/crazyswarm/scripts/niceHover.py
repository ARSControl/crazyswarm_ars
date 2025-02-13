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
    #square with one central crazyflie
    pos_array = np.array([[-0.7, 0.5, 0.5],[0.7, 0.5, 0.5],[0.5, -0.5, 0.5],[-0.5, -0.5, 0.5],[0.0,0.0,0.7]])
    for cf in allcfs.crazyflies:
        cf.goTo(pos_array[i], 0, 10.0)
        i += 1
    #go to (position,rotation,time interval)
    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
