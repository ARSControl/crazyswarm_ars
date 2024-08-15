"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

import numpy as np
import random
from pathlib import Path
import math
import os, sys
from scipy.optimize import minimize

from copy import deepcopy as dc
from sklearn.mixture import GaussianMixture

import torch
import torch.nn as nn

# import custom lib
libpath = Path.home() / "cnn_coverage"
print("dirnaem: ", (str(libpath)))
sys.path.append(str(libpath/"scripts"))
from utils import *
from models import *

# import crazyswarm
cfpath = Path.home() / "crazyswarm/ros_ws/src/crazyswarm/scripts"
sys.path.append(str(cfpath))
from pycrazyswarm import Crazyswarm

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Device: ", device)
model = SimpleCNN().to(device)
model.load_state_dict(torch.load(libpath/"trained_models/2d_param_cnn.pt", map_location=device))
model.eval()

np.random.seed(0)

ROBOTS_NUM = 8
ROBOT_RANGE = 3.0
AREA_W = 10.0
TARGETS_NUM = 4
SAMPLES_NUM = 100
STD_DEV = 1.0
MAX_STEPS = 200
SAFETY_DIST = 1.5
CONVERGENCE_TOLERANCE = 0.1

targets = -0.5*(AREA_W-2) + (AREA_W-2)*np.random.rand(TARGETS_NUM, 2)
# targets = np.array([[-2.5, -2.5], [2.5, 2.5]])
print("Targets shape:  ", targets.shape)
print("TArget 0: ", targets[0])
print("Target 1: ", targets[1])
samples = np.zeros((TARGETS_NUM, SAMPLES_NUM, 2))
for k in range(TARGETS_NUM):
    std = 0.5 + 2*np.random.rand()
    samples[k, :, :] = np.random.normal(loc=targets[k], scale=std, size=(SAMPLES_NUM, 2))

# Fit GMM
samples = samples.reshape((TARGETS_NUM*SAMPLES_NUM, 2))
gmm = GaussianMixture(n_components=TARGETS_NUM, covariance_type="full", max_iter=1000)
gmm.fit(samples)
means = gmm.means_
covariances = gmm.covariances_
mix = gmm.weights_

## -------- Generate decentralized probability grid ---------
GRID_STEPS = 64
s = AREA_W/GRID_STEPS     # step
r_step = 2 * ROBOT_RANGE / GRID_STEPS


xg = np.linspace(-0.5*AREA_W, 0.5*AREA_W, GRID_STEPS)
yg = np.linspace(-0.5*AREA_W, 0.5*AREA_W, GRID_STEPS)
Xg, Yg = np.meshgrid(xg, yg)
Xg.shape
print(Xg.shape)

Z = gmm_pdf(Xg, Yg, means, covariances, mix)
Z = Z.reshape(GRID_STEPS, GRID_STEPS)
Zmax = np.max(Z)
Z = Z / Zmax



#  CF params
TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
sleepRate = 10

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)

    converged = False
    timestep = 0
    xg_i = np.linspace(-ROBOT_RANGE, ROBOT_RANGE, GRID_STEPS)
    yg_i = np.linspace(-ROBOT_RANGE, ROBOT_RANGE, GRID_STEPS)
    Xg_i, Yg_i = np.meshgrid(xg_i, yg_i)
    robots = np.zeros((ROBOTS_NUM, 2))
    vels = np.zeros_like(robots)
    robots_hist = np.zeros((1, ROBOTS_NUM, 2))
    while not converged:
        print("Step: ", timestep)
        converged = True
        # Collect all robots positions
        for i in range(ROBOTS_NUM):
            robots[i, :] = allcfs.crazyflies[i].position()[:-1]
        print("Positions: ")
        print(robots)
        robots_hist = np.concatenate((robots_hist, np.expand_dims(robots, 0)), 0)

        
        for idx in range(ROBOTS_NUM):
            p_i = robots[idx]
            Z_i = gmm_pdf(Xg_i, Yg_i, means-p_i, covariances, mix)
            Z_i = Z_i.reshape(GRID_STEPS, GRID_STEPS)
            Z_i = Z_i / np.max(Z_i)

            neighs = np.delete(robots, idx, 0)
            local_pts = neighs - p_i

            # Remove undetected neigh
            undetected = []
            for i in range(local_pts.shape[0]):
                if local_pts[i, 0] < -ROBOT_RANGE or local_pts[i, 0] > ROBOT_RANGE or local_pts[i, 1] < -ROBOT_RANGE or local_pts[i, 1] > ROBOT_RANGE:
                    undetected.append(i)
            
            local_pts = np.delete(local_pts, undetected, 0)

            img_i = dc(Z_i)
            for i in range(GRID_STEPS):
                for j in range(GRID_STEPS):
                    # jj = GRID_STEPS-1-j
                    p_ij = np.array([-ROBOT_RANGE+j*r_step, -ROBOT_RANGE+i*r_step])
                    # print(f"Point ({i},{j}): {p_ij}")
                    for n in local_pts:
                        if np.linalg.norm(n - p_ij) <= SAFETY_DIST:
                            img_i[i, j] = -1.0

                    # Check if outside boundaries
                    p_w = p_ij + p_i
                    if p_w[0] < -0.5*AREA_W or p_w[0] > 0.5*AREA_W or p_w[1] < -0.5*AREA_W or p_w[1] > 0.5*AREA_W:
                        img_i[i, j] = -1.0
            img_in = torch.from_numpy(img_i).unsqueeze(0).unsqueeze(0)
            img_in = img_in.to(torch.float).to(device)
            vels_i = model(img_in) * r_step
            vels_i = vels_i.cpu().detach().numpy()
            vels[idx, :] = vels_i
            if np.linalg.norm(vels_i) > CONVERGENCE_TOLERANCE:
                converged = False
        
        print("Velocities: ")
        print(vels)

        for t in range(TARGETS_NUM):
            plt.scatter(targets[t,0], targets[t,1], s=14, c='r', marker='x')

        for i in range(ROBOTS_NUM):
            v = np.array([vels[i, 0], vels[i, 1], 0.0])
            allcfs.crazyflies[i].cmdVelocityWorld(v, yawRate=0)
        timeHelper.sleepForRate(sleepRate)

        if timestep > MAX_STEPS:
            break
        else:
            timestep += 1




    










    allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)

    robots_hist = robots_hist[1:]
    for i in range(ROBOTS_NUM):
        plt.plot(robots_hist[:, i, 0], robots_hist[:, i, 1])
    for t in range(TARGETS_NUM):
        plt.scatter(targets[t,0], targets[t,1], s=14, c='r', marker='x')
    
    plt.show()

    fig, ax = plt.subplots(1, 1, figsize=(8,8))
    plot_occgrid(Xg, Yg, Z, ax=ax)
    for i in range(ROBOTS_NUM):
        ax.plot(robots_hist[:, i, 0], robots_hist[:, i, 1])
        ax.scatter(robots_hist[-1, i, 0], robots_hist[-1, i, 1], s=18, marker='x')
    plt.show()

if __name__ == "__main__":
    main()
