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
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

#ROS2 Node Imports
from rclpy.node import Node
import rclpy
from message_filters import TimeSynchronizer, Subscriber

#Transform Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class LocalizationSubscriber(Node):

    def __init__(self):
        super().__init__('localization_subscriber')

        self.onboard_subscription = Subscriber(self, PoseStamped, "/cf1/pose")
        self.vicon_subscription = Subscriber(self, PoseStamped, "/cf2/pose")

        self.tss_localization = TimeSynchronizer([self.onboard_subscription, self.vicon_subscription], queue_size=0.01)
        self.tss_localization.registerCallback(self.diff_callback)
    # def onboard_callback(self, frame):
    #         # print(stamped.pose.position.x)
    #         self.get_logger().info('Onboard Estimation: (%s)' % frame.pose.position.x)  # CHANGE

    # def vicon_callback(self, frame):
    #         # print(stamped.pose.position.x)
    #         self.get_logger().info('Vicon Estimation: (%s)' % frame.pose.position.x)  # CHANGE

    def diff_callback(onboard, vicon):
        self.get_logger().info('Onboard Estimation: (%s)' % onboard.pose.position.x)
        self.get_logger().info('Vicon Estimation: (%s)' % vicon.pose.position.x)

def main(args=None):
    rclpy.init(args=args)
    # l_sub = LocalizationSubscriber()
    v_sub = LocalizationSubscriber()

    # rclpy.spin(l_sub)
    rclpy.spin(v_sub)

    v_sub.destroy_node()
    # l_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
