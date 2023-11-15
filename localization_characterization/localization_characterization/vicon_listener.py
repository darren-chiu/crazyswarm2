#!/usr/bin/env python
from pathlib import Path

#General imports
import math
import numpy as np

#Crazyflie Imports
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from crazyflie_interfaces.msg import LogDataGeneric  
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

#ROS2 Node Imports
import rclpy
from rclpy.node import Node

#Transform Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ViconSubscriber(Node):

    def __init__(self):
        super().__init__('vicon_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,                                               # CHANGE
            '/tf',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, vicon_frame):
        print(vicon_frame.transform)
        # self.get_logger().info('RVIZ Estimation: (%s)' % vicon_frame.transform.translation.x)  # CHANGE


def main(args=None):
    rclpy.init(args=args)
    # l_sub = LocalizationSubscriber()
    v_sub = ViconSubscriber()

    # rclpy.spin(l_sub)
    rclpy.spin(v_sub)

    v_sub.destroy_node()
    # l_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
