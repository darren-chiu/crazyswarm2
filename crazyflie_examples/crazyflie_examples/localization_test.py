#!/usr/bin/env python
from pathlib import Path

import math

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from crazyflie_interfaces.FullState import pose  

class LocalizationSubscriber(Node):

    def __init__(self):
        super().__init__('localization_subscriber')
        self.subscription = self.create_subscription(
            pose,                                               # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num)  # CHANGE

def main(args=None):
    rclpy.init(args=args)
    l_sub = LocalizationSubscriber()
    l_sub.spin(LocalizationSubscriber)
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    trajs = []
    n = 2  # number of distinct trajectories

    for i in range(n):
        traj = Trajectory()
        traj.loadcsv(Path(__file__).parent / f'data/multi_trajectory/traj{i}.csv')
        trajs.append(traj)

    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        for idx, cf in enumerate(allcfs.crazyflies):
            cf.uploadTrajectory(0, 0, trajs[idx % len(trajs)])

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(3.0)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0.0, 0.0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)

        allcfs.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(max([t.duration for t in trajs]) * TIMESCALE + 2.0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)

    l_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
