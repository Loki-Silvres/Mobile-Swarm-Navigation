import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import math
from std_msgs.msg import String
class get_data():
    def __init__(self,bot_name):
        self.node = Node(bot_name)
        self.name = bot_name
        self.publisher = self.node.create_publisher(PoseStamped,'/'+bot_name+'/goal_pose',10)

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_pose_node')
        
        self.declare_parameter('num_bots', 5) 
        self.num_bots = self.get_parameter('num_bots').value
        self.odom_publisher_functions = []
        self.ns = 'bot_'
        
        for i in range(self.num_bots):
            self.odom_publisher_functions.append(get_data(self.ns+str(i)))
        
        self.goal_pose_subscription = self.create_subscription(    String, 
            '/scheduled_goals', 
            self.goal_callback, 
            10
        )
        timer_period = 0.1

        self.timer = self.create_timer(timer_period,self.timer_callback)

        self.result = ""


    def goal_callback(self,goal_string: String):
        input_string = goal_string.data
        input_string = input_string.replace(" ", "")

        # Step 2: Split by semicolon to separate groups
        groups = input_string.split(";")
        self.result = [sub.split(',') for sub in groups]

        # Step 3: Convert each group into a list of integers
        # self.result = [list(map(int, group.split(","))) for group in groups if group]
        # self.get_logger().info(self.result)
    def timer_callback(self):
        goal = PoseStamped()
        for data in self.result:
            if not data[0]=='':
                    
                goal.header.frame_id = 'map'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = float(data[1])
                goal.pose.position.y = float(data[2])
                goal.pose.position.z = float(data[3])
                goal.pose.orientation.x = float(data[4])
                goal.pose.orientation.y = float(data[5])
                goal.pose.orientation.z = float(data[6])
                goal.pose.orientation.w = float(data[7])

                self.odom_publisher_functions[int(data[0])].publisher.publish(goal)



def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
