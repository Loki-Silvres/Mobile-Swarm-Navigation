#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import math
from std_msgs.msg import String
import queue
import csv
from std_msgs.msg import Header
import os

def filter_rows(file_path, target_string):
    with open(file_path, mode='r', newline='\n') as file:
        csv_reader = csv.reader(file)
        filtered_rows = []
        
        for row in csv_reader:
            if row[2] == target_string:
                filtered_rows.append(row)
        
        return filtered_rows

def remove_row_by_columns(file_path, col1_value, col2_value):
    """
    Removes rows from a CSV file where the first and second columns match the given values.

    Args:
        file_path (str): Path to the CSV file.
        col1_value: Value to match in the first column.
        col2_value: Value to match in the second column.
    """
    temp_file = file_path + '.tmp'

    with open(file_path, mode='r', newline='') as input_file, open(temp_file, mode='w', newline='\n') as output_file:
        reader = csv.reader(input_file)
        writer = csv.writer(output_file)

        header = next(reader, None)
        if header:
            writer.writerow(header)

        for row in reader:
            if not (row[0] == str(col1_value) and row[1] == str(col2_value)):
                writer.writerow(row)

    os.replace(temp_file, file_path)


class Interpreter_node(Node):
    def __init__(self):
        super().__init__('interpreter_node')
        self.data_queue = queue.Queue(maxsize=0)
        self.listener=self.create_subscription(String,'/get_object',self.listener_callback,10)
        self.timer = self.create_timer(0.1,self.send_goal)
        self.filename = '/home/pacman/semantic_database.csv'
        self.goal_sender = self.create_publisher(PoseStamped,'/schedule_goal',10)


    def listener_callback(self,input_data:String):
        self.get_logger().info(input_data.data)
        self.data_queue.put(input_data.data)

    def send_goal(self):

        if self.data_queue.qsize() > 0:
            filtered = filter_rows(self.filename,self.data_queue.get())
            if(len(filtered)>0):

                remove_row_by_columns(self.filename,filtered[0][0],filtered[0][1])
                goal_pose = PoseStamped()

                # Set the header
                goal_pose.header = Header()
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.header.frame_id = 'map'

                # Set position (x, y, z)
                goal_pose.pose.position.x = float(filtered[0][3])
                goal_pose.pose.position.y = float(filtered[0][4])
                goal_pose.pose.position.z = float(filtered[0][5])

                # Set default orientation (x, y, z, w)
                goal_pose.pose.orientation.x = 0.0
                goal_pose.pose.orientation.y = 0.0
                goal_pose.pose.orientation.z = 0.0
                goal_pose.pose.orientation.w = 1.0 

                self.get_logger().info('assigning job....')
                self.goal_sender.publish(goal_pose)
            

def main(args=None):
    rclpy.init(args=args)

    node = Interpreter_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print('out')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
