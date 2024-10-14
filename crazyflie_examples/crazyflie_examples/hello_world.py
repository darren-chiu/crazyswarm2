"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm
import numpy as np
import math

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5
z_height = 0.75
FLIGHT_TIME = 6.0
YAW = 0


def main():

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    cf.setParam('usd.logging', 1)

    cf.takeoff(targetHeight=z_height, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
    # cf.land(targetHeight=0.04, duration=2.5)
    # cf.goTo(np.array([0,0,1.2]), YAW, TAKEOFF_DURATION)
    timeHelper.sleep(HOVER_DURATION)
    # cf.goTo(np.array([0,0,1.0]), math.pi/2, TAKEOFF_DURATION)
    # timeHelper.sleep(HOVER_DURATION+1.0)
    # # timeHelper.sleep(TAKEOFF_DURATION)
    # timeHelper.sleep(HOVER_DURATION)
    # print('press button to move to origin...')
    # swarm.input.waitUntilButtonPressed()
    cf.goTo(np.array([0.0, -8.0, z_height]), 0, FLIGHT_TIME)
    # cf.goTo(np.array([3.0, 0.0, z_height]), 0, FLIGHT_TIME)
    
    # while not timeHelper.isShutdown():
    #     t = timeHelper.time() - start_time

    #     e = traj.eval(t)
    #     cf.cmdFullState(
    #         e.pos + np.array(cf.initialPosition),
    #         e.vel,
    #         e.acc,
    #         e.yaw,
    #         e.omega)

    #     timeHelper.sleepForRate(rate)
    # # # cf.goTo(np.array([4.0, 0.0, z_height]), 0, 6.0, relative=True)
    # timeHelper.sleep(FLIGHT_TIME*2)
    timeHelper.sleep(FLIGHT_TIME)
    cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(0.5)
    cf.setParam('usd.logging', 0)


if __name__ == '__main__':
    main()
