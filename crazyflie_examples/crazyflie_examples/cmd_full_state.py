#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def executeTrajectory(timeHelper, cf, trajpath, rate=100, offset=np.zeros(3)):
    traj = Trajectory()
    traj.loadcsv(trajpath)

    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > traj.duration:
            break

        e = traj.eval(t)
        cf.cmdFullState(
            e.pos + np.array(cf.initialPosition) + offset,
            e.vel,
            e.acc,
            e.yaw,
            e.omega)

        timeHelper.sleepForRate(rate)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    rate = 100.0
    Z = 0.65
    cf.setParam("kalman.initialX", 0)
    cf.setParam("kalman.initialY", 0)
    cf.setParam("kalman.resetEstimation", 1)
    cf.setParam('usd.logging', 1)
    timeHelper.sleep(0.1)


    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(5)

    executeTrajectory(timeHelper, cf,
                      Path(__file__).parent / 'data/figure8.csv',
                      rate,
                      offset=np.array([0, 0, 0.65]))
    timeHelper.sleep(2)
    cf.notifySetpointsStop()
    cf.land(targetHeight=0.0, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    cf.setParam('usd.logging', 0)

if __name__ == '__main__':
    main()