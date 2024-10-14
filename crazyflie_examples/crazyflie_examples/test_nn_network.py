"""Script that tests the ToF NN Network"""

from crazyflie_py import Crazyswarm
import numpy as np
import math
import cv2 

#TAKEOFF PARAMETERS
TAKEOFF_DURATION = 1.0
HOVER_DURATION = 5.0
Z = 0.65

#FLIGHT PARAMETERS
FLIGHT_TIME = 8.0
X_DIST = 4.0
YAW = 0


# MULI DRONE
ENABLE_MULTI_DRONE = False
NUM_DRONES = 3
init_x = [0.0, 0.0, 0.0, -1.0]
init_y = [1.0, 0.0, -1.0, 0.0]


swarm = Crazyswarm()
timeHelper = swarm.timeHelper

def setInitialPos(allcfs):
     for i in range(NUM_DRONES):
        allcfs.crazyfliesById[i].setParam("kalman.initialY", init_y[i])
        allcfs.crazyfliesById[i].setParam("kalman.initialX", init_x[i])
        allcfs.crazyfliesById[i].setParam("kalman.resetEstimation", 1)
        timeHelper.sleep(0.1)

def main():
    

    if (ENABLE_MULTI_DRONE):
        allcfs = swarm.allcfs
        setInitialPos(allcfs)

        allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION + 0.5)
        timeHelper.sleep(HOVER_DURATION)
        
        for cf in allcfs.crazyflies:
            cf.setParam('paramNN.thrust_offset', 0)
            cf.setParam('usd.logging', 1)
        
        timeHelper.sleep(0.1)
        
        for cf in allcfs.crazyflies:
            cf.goTo(np.array([X_DIST, 0, Z]), 0, FLIGHT_TIME, relative=False)

        timeHelper.sleep(FLIGHT_TIME*1.2)
        
        for cf in allcfs.crazyflies:
            cf.goTo(np.array([0.0, 0.0, Z]), 0, FLIGHT_TIME, relative=False)
            
        timeHelper.sleep(FLIGHT_TIME*1.2)
        
        # timeHelper.sleep(15)
        
        allcfs.land(targetHeight=0.01, duration=TAKEOFF_DURATION)
        timeHelper.sleep(0.5)
        
        for cf in allcfs.crazyflies:
            cf.setParam('usd.logging', 0)
    else:
        cf = swarm.allcfs.crazyflies[0]
        cf.setParam("kalman.initialX", 0)
        cf.setParam("kalman.initialY", 0)
        
        cf.setParam("kalman.resetEstimation", 1)
        
        cf.setParam('usd.logging', 1)
        cf.setParam('paramNN.thrust_offset', 0)
        # cf.setParam('paramNN.thrust_offset', 14000)

        timeHelper.sleep(0.1)

        cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION)
        
        timeHelper.sleep(HOVER_DURATION)
        
        cf.goTo(np.array([X_DIST,0,Z]), YAW, FLIGHT_TIME)
        timeHelper.sleep(FLIGHT_TIME + 2)
        cf.goTo(np.array([0,0,Z]), YAW, FLIGHT_TIME)
        timeHelper.sleep(FLIGHT_TIME + 2)
        
        cf.land(targetHeight=0.01, duration=TAKEOFF_DURATION)
        timeHelper.sleep(0.5)
        cf.setParam('usd.logging', 0)


if __name__ == '__main__':
    main()
