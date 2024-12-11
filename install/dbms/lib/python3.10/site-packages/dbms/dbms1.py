import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
import csv
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory
class DBMS(Node):
    def __init__(self):
        super.__init__('my_dbs_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.source_frame = 'bot_0'  # Replace with your source frame
        self.target_frame = 'map'  # Replace with your target frame

        self.subscription = self.create_subscription(String,'/detected_object',self.detected_callback,10)

        self.file = open(os.path.join(get_package_share_directory('dbms'),'data_base','object_detected_database.csv'), mode='a', newline='')
        self.writer = csv.writer(self.file)
        

    def transform_point(self, x, y, z):
        try:
            # Create a PointStamped message in the source frame
            point = PointStamped()
            point.header.frame_id = self.source_frame
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = x
            point.point.y = y
            point.point.z = z

            # Look up the transform and apply it
        
            transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.time.Duration(seconds=1.0)
                )
            return do_transform_point(point, transform)
        except Exception as e:
            self.get_logger().error(f"Error transforming point: {e}")
            return None
    

    def detected_callback(self,input_data: String):
        input_list = input_data.data.split(',')
        transformed_point = self.transform_point(input_list[1],input_list[2],input_list[3])
        self.writer.writerow([input_list[0],transformed_point.point.x,transformed_point.point.y])

