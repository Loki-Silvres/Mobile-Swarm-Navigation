import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import math
from std_msgs.msg import String
import queue
idle_bots = []
goal_dict = dict()
class get_data():
    def __init__(self,bot_name):
        self.node = Node(bot_name)
        self.name = bot_name
        self.bot_id = None
        self.logger = rclpy.logging.get_logger(bot_name)
        self.x = 0
        self.y = 0
        # self.publisher = self.node.create_publisher(PoseStamped,'/'+bot_name+'/goal_pose',10)
        self.state = 0
        self.goal_x = 0
        self.goal_y = 0
        self.distance = 0
        self.task_id = 0


        
        # 0 idle 1 occupied 2 exploration
        
    def callback(self,msg:Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.distance = math.sqrt((self.x-self.goal_x)**2+(self.y-self.goal_y)**2)
        # print(self.distance)
        if (self.distance<=0.6):
            self.state = 0
            if not self.bot_id in idle_bots:
                    
                self.logger.info(f'{self.name} is idle now')
                # print(f'{self.name} is idle now')
                idle_bots.append(self.bot_id)
            try:

                del goal_dict[(self.goal_x,self.goal_y)]
            except Exception as e:
                pass
        # Extracting the orientation in quaternion
        # orientation = msg.pose.pose.orientation
        # qx = orientation.x
        # qy = orientation.y
        # qz = orientation.z
        # qw = orientation.w
        # self.logger.info(f'Robot Pose: x={x:.2f}, y={y:.2f}, quaternion=({qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f})')

class OdomPoseNode(Node):
    def __init__(self):
        super().__init__('odom_pose_node')
        
        self.declare_parameter('num_bots', 5) 
        self.num_bots = self.get_parameter('num_bots').value
        self.task_id=1
        self.odom_subscibers = []
        self.odom_subsciber_functions = []
        self.ns = 'bot_'
        self.robot_task_map = dict()
        self.task_robot_map = dict()
        self.data_queue = queue.Queue(maxsize=0)

        # self.robot_task_map = list()
        # self.task_robot_map = list()
        
        for i in range(self.num_bots):
            self.odom_subsciber_functions.append(get_data(self.ns+str(i)))
            self.odom_subsciber_functions[i].bot_id = i
            self.odom_subscibers.append(self.create_subscription(Odometry,'/'+self.ns+str(i)+'/amcl_pose',self.odom_subsciber_functions[i].callback,10))
            self.robot_task_map[i]=None
            idle_bots.append(i)
        
        self.goal_pose_subscription = self.create_subscription(PoseStamped, 
            '/schedule_goal', 
            self.goal_callback, 
            10
        )
        duration = 0.1
        self.goal_publisher = self.create_publisher(String,'/scheduled_goals',10)
        self.create_timer(duration,self.goal_pub_callback)
        self.create_timer(duration,self.processor)

    def goal_callback(self,goal_msg: PoseStamped):

        goal_x = goal_msg.pose.position.x
        goal_y = goal_msg.pose.position.y
        goal_tup = (goal_x, goal_y)
        if goal_tup in goal_dict:
            print(f'job {goal_tup} already assigned to bot_{self.task_robot_map[goal_dict[goal_tup][0]]}')
        else:

            my_goal_tup = (goal_msg,self.task_id)
            self.task_id +=1
            self.data_queue.put(my_goal_tup)



        # goal_x = goal_msg.pose.position.x
        # goal_y = goal_msg.pose.position.y
        # goal_tup = (goal_x, goal_y)
        # if not goal_tup in goal_dict.keys():
        #     goal_dict[goal_tup] = [False, self.task_id, None]
        #     task_id = self.task_id
        #     self.task_id+=1
        # min = 1000000
        # name = 'nothing'
        # min_i = 100000
        # if goal_dict[goal_tup][0]:
        #     # self.odom_subsciber_functions[self.task_robot_map[goal_dict[goal_tup][1]]].publisher.publish(goal_msg)
        #     print('hi')
        # else:

            
        #     for i in range(self.num_bots):
        #         distance = math.sqrt((goal_x-self.odom_subsciber_functions[i].x)**2 + (goal_y-self.odom_subsciber_functions[i].y)**2)
        #         if distance < min or self.odom_subsciber_functions[i].state ==0 or self.odom_subsciber_functions[i].state == 2:
        #             min = distance
        #             name = self.odom_subsciber_functions[i].name
        #             min_i = i
        #         # if self.odom_subsciber_functions[i].state ==1 and  self.odom_subsciber_functions[i].task_id == goal_dict[goal_tup][1]:
        #         #     self.odom_subsciber_functions[i].publisher.publish(goal_msg)
        #     self.get_logger().info(f'closest is {name}')
        #     self.robot_task_map[min_i] = task_id
        #     self.task_robot_map[task_id] = min_i

        #     self.get_logger().info(f'closest is {name} and job assgined')
        #     self.odom_subsciber_functions[min_i].goal_x = goal_x
        #     self.odom_subsciber_functions[min_i].goal_y = goal_y
            
        #     # self.odom_subsciber_functions[min_i].publisher.publish(goal_msg)
        #     goal_dict[goal_tup][0]=True
        #     goal_dict[goal_tup][2]=goal_msg
            
        #     self.odom_subsciber_functions[min_i].state = 1
        #     self.odom_subsciber_functions[min_i].task_id = goal_dict[goal_tup][1]
        # if not goal_dict[goal_tup][0]:
        #     self.queue.put()
            
            # if not goal_dict[goal_tup][0]:

            #     self.get_logger().info(f'closest is {name} and job assgined')
            #     self.odom_subsciber_functions[min_i].goal_x = goal_x
            #     self.odom_subsciber_functions[min_i].goal_y = goal_y
                
            #     self.odom_subsciber_functions[min_i].publisher.publish(goal_msg)
            #     goal_dict[goal_tup][0]=True
            #     self.odom_subsciber_functions[min_i].state = 1
            #     self.odom_subsciber_functions[min_i].task_id = goal_dict[goal_tup][1]
            #     self.robot_task_map[min_i] = self.task_id
            #     self.task_robot_map[self.task_id] = min_i

            # TODO state checking and state updation

    def goal_pub_callback(self):

        out_str = ""
        send_str = String()
        for k,v in goal_dict.items():
            out_str = out_str + str(self.task_robot_map[v[0]]) + ','+self.convert_posestamped_to_string(v[1]) + ';'
            # self.get_logger().info(v)
        send_str.data = out_str
        print(out_str)
        self.goal_publisher.publish(send_str)
    def convert_posestamped_to_string(self, pose_stamped):
        position = pose_stamped.pose.position
        orientation = pose_stamped.pose.orientation

    # Format into a string
        return f"{position.x},{position.y},{position.z},{orientation.x},{orientation.y},{orientation.z},{orientation.w}"

    def processor(self):
        if self.data_queue.qsize() != 0 and len(idle_bots)!=0:
            goal_data = self.data_queue.get()
            goal_tup = (goal_data[0].pose.position.x,goal_data[0].pose.position.y)
            goal_x = goal_tup[0]
            goal_y = goal_tup[1]
            
            task_id = goal_data[1]
            min = 1000000
            min_i = 0
            for i in idle_bots:
                distance = math.sqrt((goal_x-self.odom_subsciber_functions[i].x)**2 + (goal_y-self.odom_subsciber_functions[i].y)**2)
                if distance < min:
                    min = distance 
                    min_i = i
                    name = self.odom_subsciber_functions[i].name
            
            self.get_logger().info(f'closest is {name}')
            self.robot_task_map[min_i] = task_id
            self.task_robot_map[task_id] = min_i

            self.get_logger().info(f'closest is {name} and job assgined')
            self.odom_subsciber_functions[min_i].goal_x = goal_x
            self.odom_subsciber_functions[min_i].goal_y = goal_y
            
            # self.odom_subsciber_functions[min_i].publisher.publish(goal_msg)
            goal_dict[goal_tup]= [task_id,goal_data[0]]
            
            self.odom_subsciber_functions[min_i].state = 1
            self.odom_subsciber_functions[min_i].task_id = goal_dict[goal_tup][0]
            self.robot_task_map[min_i] = self.task_id
            self.task_robot_map[self.task_id] = min_i
            idle_bots.remove(min_i)





def main(args=None):
    rclpy.init(args=args)
    node = OdomPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
