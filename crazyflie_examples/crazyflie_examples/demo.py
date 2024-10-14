#!/usr/bin/env python
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_py import Crazyswarm
import numpy as np
from pathlib import Path
import math

def main():
    TAKEOFF_DURATION = 2.5
    FLIGHT_DURATION = 30
    z_height = 1.7
    YAW = 0.0
    rate=45
    

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0] 
     
    cf.takeoff(targetHeight=z_height, duration=TAKEOFF_DURATION)
    # cf.goTo(np.array([0, 0, 1.1]), 0, 1.0)
    # timeHelper.sleep(takeoff_duration+0.5)
    timeHelper.sleep(TAKEOFF_DURATION)
    cf.goTo(np.array([0,0,z_height]), YAW, TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    print('press button to move to origin...')
    swarm.input.waitUntilButtonPressed()

    # pos = np.array(cf.initialPosition) + np.array([4.0, 0.0, 0.5])
    # cf.goTo(pos, 0, 1.0)

    # print('Ctrl-C to stop...')
    print("Sending Full State")
    # while(1):
    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        # cf.goTo(np.array([1.0,0,z_height]), YAW, TAKEOFF_DURATION)
        t = timeHelper.time() - start_time
        if t > FLIGHT_DURATION:
            break

        # cf.cmdFullState([np.sin(t),0.0,z_height], 
        #         [np.sin(t),0.0,0.0], 
        #         [0.0,0.0,0.0], 
        #         YAW, 
        #         [0.0,0.0,0.0])
        # cf.cmdPosition([np.sin(t*(rate/4)),np.cos(t*(rate/4)),z_height], YAW)
        cf.cmdPosition([(0.0),(0.0),z_height], YAW)
        # timeHelper.sleep(1)
        # cf.goTo(np.array([-1.0,0,z_height]), YAW, TAKEOFF_DURATION)
        # cf.cmdFullState([2.0,2.0,z_height], 
        #         [0.0,0.0,0.0], 
        #         [0.0,0.0,0.0], 
        #         0.0, 
        #         [0.0,0.0,0.0])
        timeHelper.sleepForRate(rate)
    timeHelper.sleep(1)
    print("Landing....")
    # cf.land(targetHeight=0.04, duration=2.5)
    cf.cmdFullState([0.0,0.0,0.2], 
                [0.0,0.0,0.0], 
                [0.0,0.0,0.0], 
                0.0, 
                [0.0,0.0,0.0])
    


if __name__ == '__main__':
    main()
