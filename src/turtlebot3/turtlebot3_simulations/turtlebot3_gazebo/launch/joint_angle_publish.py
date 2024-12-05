#!/usr/bin/env python3

# Import necessary libraries
import rclpy  # ROS 2 Python client library for writing nodes
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from sensor_msgs.msg import JointState  # Message type for publishing joint states
import math  # For mathematical calculations (e.g., trigonometry)

# Function to compute joint angles for a 2-link robotic arm
def compute_joint_angle(x, y, z, initial_angles=(0.0, 0.0 , 0.0 , 0.0)):
    # Arm link lengths (specific to your robot arm design)
    link1_length = 131.0  # Length of the first arm segment (e.g., shoulder to elbow)
    link2_length = 192.0  # Length of the second arm segment (e.g., elbow to wrist)

    # Calculate theta1 (rotation of the base joint in the XY plane)
    r = math.sqrt(x**2 + y**2)  # Calculate the horizontal distance from the base to the target
    theta1 = math.atan2(y, x)  # Compute the angle to rotate the base joint
    theta1 -= initial_angles[0]  # Subtract initial offset if provided

    # Calculate the total distance from the base to the target point
    d = math.sqrt(r**2 + z**2)
    
    # Check if the target point is reachable
    if d > link1_length + link2_length:
        raise ValueError("Target position is out of the robot's reach.")

    # Calculate the angle of the target point relative to the horizontal plane
    phi = math.atan2(z, r)

    # Use the law of cosines to find angles for the arm joints
    cos_alpha = (link1_length**2 + d**2 - link2_length**2) / (2 * link1_length * d)
    cos_beta = (link1_length**2 + link2_length**2 - d**2) / (2 * link1_length * link2_length)
    
    # Ensure cosine values are valid (should be between -1 and 1)
    if not (-1 <= cos_alpha <= 1) or not (-1 <= cos_beta <= 1):
        raise ValueError("Invalid position due to cosine limits.")

    # Calculate the angles using arccos (inverse cosine)
    alpha = math.acos(cos_alpha)
    beta = math.acos(cos_beta)

    # Compute joint angles
    theta2 = math.pi/2 - (phi + alpha)  # Angle for the second joint
    theta3 = math.pi/2 - beta  # Angle for the third joint
    theta4 = (phi + alpha + beta) - math.pi  # Angle for the end-effector (adjust if needed)

    # Subtract initial offsets if provided
    theta2 -= initial_angles[1]
    theta3 -= initial_angles[2]
    theta4 -= initial_angles[3]

    # Return all joint angles in radians
    return [theta1, theta2, theta3, theta4]

# Define a ROS 2 node to publish joint states
class JointStatePublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'custom_joint_publisher'
        super().__init__('custom_joint_publisher')

        # Create a publisher to the '/joint_states' topic
        self.publisher_ = self.create_publisher(JointState, '/bot_0/joint_states', 10)
        # Create a timer to periodically call the publish_joint_states method
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Declare parameters for target coordinates (default values are when joints are at 0.0)
        self.declare_parameter('x', 192.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 131.0)

        # Retrieve the parameter values provided at runtime
        self.x_coordinate = self.get_parameter('x').get_parameter_value().double_value
        self.y_coordinate = self.get_parameter('y').get_parameter_value().double_value
        self.z_coordinate = self.get_parameter('z').get_parameter_value().double_value
        
        # ros2 run example usage for passing parameters
        # ros2 run turtlebot3_manipulation_description joint_angle_publish.py --ros-args -p x:=0.1 -p y:=-0.1 -p z:=0.05 -p config:="elbow-up"

        # Compute the joint angles based on the target coordinates
        self.joint_angles = compute_joint_angle(self.x_coordinate, self.y_coordinate, self.z_coordinate)

        # Define the initial positions of the joints
        self.joint_positions = {
            'bot_0joint1': self.joint_angles[0],  # Base joint angle
            'bot_0joint2': self.joint_angles[1],  # Shoulder joint angle
            'bot_0joint3': self.joint_angles[2],  # Elbow joint angle
            'bot_0joint4': self.joint_angles[3],  # End-effector orientation
        }

    # Method to publish the current joint states
    def publish_joint_states(self):
        msg = JointState()  # Create a JointState message
        msg.header.stamp = self.get_clock().now().to_msg()  # Add a timestamp
        msg.name = list(self.joint_positions.keys())  # Add joint names
        msg.position = list(self.joint_positions.values())  # Add joint angles
        self.publisher_.publish(msg)  # Publish the message

# Main function to run the node
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 communication
    node = JointStatePublisher()  # Create an instance of the node
    rclpy.spin(node)  # Keep the node running and responding to events
    node.destroy_node()  # Clean up the node when shutting down
    rclpy.shutdown()  # Shut down ROS 2 communication

# Entry point of the script
if __name__ == '__main__':
    main()
