import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import LogDataGeneric
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import csv
import os
import ast

context = rclpy.context.Context()
experiment_name = 'experiment_data/hover/'

class PoseLoggerNode(Node):
    def __init__(self):
        super().__init__('pose_logger')
        
        self.topics = []
        
        for i in range(3):
            self.topics.append(f'/cf{i}/localization')

        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)
        
        self.subscribers = {}
        
        self.data = {topic: [] for topic in self.topics}
        
        for topic in self.topics:
            # Create a subscriber for each topic
            self.subscribers[topic] = self.create_subscription(
                LogDataGeneric, 
                topic,
                lambda msg, topic=topic: self.pose_callback(msg, topic),
                self.qos_profile
            )
        
        # subscriber = self.create_subscription(PoseStamped, 
        #                                       '/rosout', 
        #                                       self.pose_callback, 
        #                                       self.qos_profile)
        # self.subscription
    def pose_callback(self, msg, topic):
        # timestamp, array_data = msg.values.split(",", 1)
        # array_data = ast.literal_eval(array_data)  # Safely evaluate the string as a Python literal
        # self.data[topic].append([int(msg.values[0]), int(msg.values[1]), int(msg.values[2])])
        timestamp = msg.header.stamp.sec  # Assuming timestamp is in the message header
        array_values = list(msg.values)  # Convert the array to a list
        print(type(array_values))
        self.data[topic].append(array_values)  # Append data with timestamp

        # print(f"Received message {topic}: x={msg.values[0]}, y={msg.values[1]}, z={msg.values[2]}")
    def save_to_csv(self):
        
        # if all(len(self.data[topic]) == num_data_points for topic in self.topics):
        # Create a CSV file for each topic
        for topic in self.topics:
            filename = experiment_name + f"{topic.strip('/').replace('/', '_')}.csv"
            
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'Data'])  # Header row
                for data_row in self.data[topic]:
                    writer.writerow(data_row)
                    
            self.get_logger().info(f"Data saved to {filename}")

def main():
    rclpy.init()
    poseloggernode = PoseLoggerNode()
    
    try:
    # print("Spinning...")
        rclpy.spin(poseloggernode)
        
    except KeyboardInterrupt:
        pass
    finally:
        poseloggernode.save_to_csv()
        poseloggernode.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()