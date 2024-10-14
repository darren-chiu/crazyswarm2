#!/usr/bin/env python

from crazyflie_py import Crazyswarm
import numpy as np


def main():
    Z = 1.25
    takeoff_duration = 1.2+Z
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=takeoff_duration)
    timeHelper.sleep(takeoff_duration + 0.5)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([-0.5, 0, Z])
        cf.goTo(pos, 0, 1.0)

    print('press button to move to origin...')
    swarm.input.waitUntilButtonPressed()

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 1.0)

    print('press button to move to offset...')
    swarm.input.waitUntilButtonPressed()

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0.5, Z])
        cf.goTo(pos, 0, 1.0)

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)


if __name__ == '__main__':
    main()
