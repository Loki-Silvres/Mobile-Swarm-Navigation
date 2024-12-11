#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

class SwarmScheduler(Node):
    def _init_(self):
        super()._init_('swarm_scheduler')
        
        # Parameters
        self.declare_parameter('num_bots', 5)  # Default 5 bots
        self.num_bots = self.get_parameter('num_bots').value
        
        # Robot state tracking
        self.robot_states = {}
        self.robot_positions = {}
        
        # Goal subscription
        self.goal_subscription = self.create_subscription(
            PoseStamped, 
            '/goal_pose', 
            self.goal_callback, 
            10
        )
        
        # Create state publishers for each bot
        self.state_publishers = {}
        for i in range(self.num_bots):
            namespace = f'bot_{i}'
            
            # State publisher
            state_topic = f'/{namespace}/task_state'
            publisher = self.create_publisher(String, state_topic, 10)
            self.state_publishers[namespace] = publisher
            
            # Initial robot state setup
            self.robot_states[namespace] = 'FREE'
            
            # Odometry subscription to track robot positions
            odom_topic = f'/{namespace}/odom'
            self.create_subscription(
                Odometry, 
                odom_topic, 
                lambda msg, ns=namespace: self.update_robot_position(msg, ns), 
                10
            )
        
        self.get_logger().info('Swarm Scheduler initialized')
    
    def update_robot_position(self, msg: Odometry, namespace: str):
        """Update robot's current position"""
        position = msg.pose.pose.position
        self.robot_positions[namespace] = position
    
    def calculate_distance(self, point1: Point, point2: Point):
        """Calculate Euclidean distance between two points"""
        return math.sqrt(
            (point1.x - point2.x)**2 + 
            (point1.y - point2.y)**2 + 
            (point1.z - point2.z)**2
        )
    
    def goal_callback(self, goal_msg: PoseStamped):
        """Handle incoming goal and assign to closest free robot"""
        goal_position = goal_msg.pose.position
        
        # Find closest free robot
        closest_robot = None
        min_distance = float('inf')
        
        for namespace, state in self.robot_states.items():
            if state == 'FREE' and namespace in self.robot_positions:
                robot_position = self.robot_positions[namespace]
                distance = self.calculate_distance(robot_position, goal_position)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_robot = namespace
        
        # Assign task to closest free robot
        if closest_robot:
            # Update robot state
            self.robot_states[closest_robot] = 'BUSY'
            
            # Publish task state
            state_msg = String()
            state_msg.data = 'BUSY'
            self.state_publishers[closest_robot].publish(state_msg)
            
            self.get_logger().info(
                f'Task assigned to {closest_robot}. '
                f'Distance to goal: {min_distance:.2f}'
            )
        else:
            self.get_logger().warn('No free robots available for task')
    
    def mark_robot_free(self, namespace: str):
        """Mark a robot as free after completing a task"""
        if namespace in self.robot_states:
            self.robot_states[namespace] = 'FREE'
            
            # Publish free state
            state_msg = String()
            state_msg.data = 'FREE'
            self.state_publishers[namespace].publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    scheduler = SwarmScheduler()
    rclpy.spin(scheduler)
    rclpy.shutdown()

if _name_ == '_main_':
    main()