"""Script that tests the ToF NN Network"""

from crazyflie_py import Crazyswarm
from pathlib import Path
import numpy as np
from crazyflie_py.uav_trajectory import Trajectory
import math
import csv

# RECORD_VICON = False
# if (RECORD_VICON):
#     # Initialize a CSV file to save positions
#     log_file = open('positions_log.csv', 'w', newline='')
#     csv_writer = csv.writer(log_file)
#     csv_writer.writerow(['time', 'drone', 'x', 'y', 'z'])


Z = 0.65
YAW = 0
TAKEOFF_DURATION = 2.5

# MULI DRONE
ENABLE_MULTI_DRONE = False

ENABLE_LOGGING = True

swarm = Crazyswarm()
timeHelper = swarm.timeHelper

# Global variables for threading and shared data
if (ENABLE_LOGGING):
    import rclpy
    import threading
    from geometry_msgs.msg import PoseStamped
    
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    
    context = rclpy.context.Context()
    # rclpy.init(context=context)
    
    # Specify topics and output CSV file
    pose_data = []
    logging_active = True
    pose_lock = threading.Lock()


class PoseLogger(Node):
    def __init__(self, topic_names):
        super().__init__('pose_logger')
        
        self.topic_names = topic_names
        self.subscribers = []
        self.processing = False
        
        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
        for topic in topic_names:
            print("Subscribed to: ", topic)
            
            subscriber = self.create_subscription(PoseStamped, topic, self.pose_callback, self.qos_profile)
            
            if subscriber is None:
                print(f"Failed to subscribe to topic: {topic}")
            else:
                self.subscribers.append(subscriber)

    def pose_callback(self, msg):
        print("POSE CALLBACK")
        """Callback for logging pose data."""
        with pose_lock:
            timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
            position = msg.pose.position
            print(f"Received: x={position.x}, y={position.y}, z={position.z}")
            
            print(topic_name, position.x, position.y, position.z)
            topic_name = msg._connection_header['topic']
            
            pose_data.append([timestamp, topic_name, position.x, position.y, position.z])  # Add topic name
            

    def shutdown(self):
        """Graceful shutdown the logger."""
        self.destroy_node()
    
def pose_logger_thread(topic_names, context):
    """Run PoseLogger in a separate thread."""
    rclpy.init(context=context)
    pose_logger = PoseLogger(topic_names)
    try:
        # while logging_active:
        #     # print("Spinning pose logger...")
        #     rclpy.spin_once(pose_logger, timeout_sec=0.1)
        rclpy.spin(pose_logger)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down pose logger")
        pose_logger.shutdown()
        rclpy.shutdown(context=context)
        
def save_pose_data_to_csv(filename):
    """Save logged pose data to a CSV file."""
    with pose_lock:
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # writer.writerow(['time', 'x', 'y', 'z'])  # Header
            writer.writerow(['time', 'topic', 'x', 'y', 'z'])
            writer.writerows(pose_data)
            
def run_formation_square(allcfs):
    X_DIST = 4.0
    FLIGHT_TIME = 8.0
    NUM_DRONES = 3
    initial_positions = np.array([[0.0, 0.0, Z], 
                    [0.0, -0.5, Z], 
                    [-0.5, -0.5, Z],
                    [-0.5, 0.0, Z]])
    
    # initial_positions = np.array([[0.0, 0.0, Z], 
    #                  [0.0, -1.0, Z], 
    #                  [-1.0, -1.0, Z],
    #                  [-1.0, 0.0, Z]])
    
    setInitialPos(allcfs, num_drones=NUM_DRONES, initial_positions=initial_positions)
    timeHelper.sleep(0.1)
    allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
    
    allcfs.goTo([0.0, -X_DIST, 0.0], YAW, FLIGHT_TIME)
    timeHelper.sleep(FLIGHT_TIME*1.2)
    
    allcfs.goTo([0.0, X_DIST, 0.0], YAW, FLIGHT_TIME)
    timeHelper.sleep(FLIGHT_TIME*1.2)
    
    allcfs.land(targetHeight=0.01, duration=TAKEOFF_DURATION)
    timeHelper.sleep(0.5)
        
def run_simple_case_A(allcfs):
    
    #TAKEOFF PARAMETERS
    HOVER_DURATION = 2.0
    
    #FLIGHT PARAMETERS
    FLIGHT_TIME = 5.0
    X_DIST = 2.0

    if (ENABLE_MULTI_DRONE):
        setInitialPos(allcfs)

        allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION + 0.5)
        timeHelper.sleep(HOVER_DURATION)

        for cf in allcfs.crazyflies:
            # cf.setParam('paramNN.thrust_offset', 0)
            cf.setParam('usd.logging', 1)
        
        timeHelper.sleep(1.0)
        
        counter = 0
        for cf in allcfs.crazyflies:
            # Configuration C
            # cf.goTo(np.array([X_DIST, 0, Z]), 0, FLIGHT_TIME, relative=False)

            # Configuration D
            pos = initial_positions[counter] + np.array([X_DIST, 0.0, 0.0])
            cf.goTo(pos, 0, FLIGHT_TIME)
            counter = counter + 1


        timeHelper.sleep(FLIGHT_TIME*1.2)
        
        # for cf in allcfs.crazyflies:
        #     cf.goTo(np.array([0.0, 0.0, Z]), 0, FLIGHT_TIME, relative=False)
            
        # timeHelper.sleep(FLIGHT_TIME*1.2)
        
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
        
        # cf.setParam('usd.logging', 1)
        # cf.setParam('paramNN.thrust_offset', 0)
        # cf.setParam('paramNN.thrust_offset', 14000)

        timeHelper.sleep(0.1)

        cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION)
        
        timeHelper.sleep(HOVER_DURATION)
        
        cf.goTo(np.array([0.0,-1.0 * X_DIST,Z]), YAW, FLIGHT_TIME)
        timeHelper.sleep(FLIGHT_TIME + 10.0)
        
        cf.land(targetHeight=0.01, duration=3.0)
        timeHelper.sleep(0.5)
        # cf.setParam('usd.logging', 0)
        
        
def run_simple_hover(allcfs):
    #TAKEOFF PARAMETERS
    HOVER_DURATION = 10.0

    if (ENABLE_MULTI_DRONE):
        num_drones = 3
        initial_positions = np.array([[0.0, 0.0, Z], 
                     [0.0, -0.5, Z], 
                     [-0.5, -0.5, Z],
                     [-0.5, 0.0, Z]])
        
        setInitialPos(allcfs, num_drones, initial_positions)

        allcfs.takeoff(targetHeight=0.75, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION)
        timeHelper.sleep(HOVER_DURATION)
        
        allcfs.land(targetHeight=0.01, duration=TAKEOFF_DURATION)
    else:
        cf = swarm.allcfs.crazyflies[0]
        cf.setParam("kalman.initialX", 0)
        cf.setParam("kalman.initialY", 0)
        
        cf.setParam("kalman.resetEstimation", 1)
        
        cf.setParam('usd.logging', 1)
        # cf.setParam('paramNN.thrust_offset', 0)
        # cf.setParam('paramNN.thrust_offset', 14000)

        timeHelper.sleep(0.1)

        cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
        timeHelper.sleep(TAKEOFF_DURATION)
        
        timeHelper.sleep(HOVER_DURATION)
        
        cf.land(targetHeight=0.01, duration=TAKEOFF_DURATION)
        timeHelper.sleep(0.5)
        cf.setParam('usd.logging', 0)

def run_uav_traj(allcfs):
    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/traj_custom.csv')
    
    TIMESCALE = 5.0
    
    for cf in allcfs.crazyflies:
            cf.setParam('usd.logging', 1)
            timeHelper.sleep(1.0)
            cf.uploadTrajectory(0, 0, traj1)
        
    allcfs.takeoff(targetHeight=0.65, duration=2.0)
    timeHelper.sleep(3.0)
    allcfs.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
    allcfs.land(targetHeight=0.1, duration=2.0)
    timeHelper.sleep(2.5)
    for cf in allcfs.crazyflies:
        cf.setParam('usd.logging', 0)
    timeHelper.sleep(0.5)

def setInitialPos(allcfs, num_drones, initial_positions):

    for i in range(num_drones):
        allcfs.crazyfliesById[i].setParam("kalman.initialY", initial_positions[i][1])
        allcfs.crazyfliesById[i].setParam("kalman.initialX", initial_positions[i][0])
        allcfs.crazyfliesById[i].setParam("kalman.initialYaw", 0)
    
        # Reset Estimation
        allcfs.crazyfliesById[i].setParam("kalman.resetEstimation", 1)
    timeHelper.sleep(0.2)

def main():
    allcfs = swarm.allcfs
    
    if (ENABLE_LOGGING):
        topic_names = ['/cf0/localization']  # Replace with your topics
        experiment_name = 'experiment_data/localization_log.csv' 
        
        global logging_active
        # rclpy.init(context=context)
    
        logger_thread = threading.Thread(target=pose_logger_thread, args=(topic_names, context))
        logger_thread.start()
    
        try:
            # Run the existing experiment logic

            # run_uav_traj
            # run_uav_traj(allcfs)
            
            # Go forward 4.0 meters
            # run_simple_case_A(allcfs)
            
            # Formation square, then go forward 4.0 meters, and return.
            # run_formation_square(allcfs)
            
            # Hover
            run_simple_hover(allcfs)
            # timeHelper.sleep(5.0)
        finally:
            # Stop logging and save the pose data
            logging_active = False
            logger_thread.join()
            save_pose_data_to_csv(experiment_name)
            print("Pose data saved to: " + experiment_name)
            
    else:
        run_formation_square(allcfs)
        # allcfs.land(targetHeight=0.01, duration=TAKEOFF_DURATION)
    


if __name__ == '__main__':
    main()


