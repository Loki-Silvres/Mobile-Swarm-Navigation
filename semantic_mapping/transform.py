import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
import csv
from ultralytics import YOLO
class MultiArrayTransformer(Node):
    def __init__(self):
        super().__init__('multiarray_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Input topic for Float32MultiArray
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/coordinates_topic',  # Replace with your Float32MultiArray topic
            self.multiarray_callback,
            10
        )
        self.source_frame = 'bot_0/camera_rgb_depth_frame'  # Replace with your source frame
        self.target_frame = 'bot_0/odom'  # Replace with your target frame
        self.model = YOLO('inter-iit1n.pt')  # Replace with your YOLO model path

        # Open the CSV file in append mode
        self.file = open('coordinates.csv', mode='a', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
                        'x','y','z','class'
                    ])
    def multiarray_callback(self, msg):
        # Extract data from Float32MultiArray
        points = msg.data  # This is a flattened array of [x, y, z, x, y, z, ...]

        for i in range(0, len(points),4):  # Assuming every 4th element is the class_id
            x, y, z,class_id = points[i], points[i+1], points[i+2], points[i+3]
            
            # Transform the point
            transformed_point = self.transform_point(x, y, z)

            if transformed_point:
                    # Write the transformed point and class_id to CSV
                    self.writer.writerow([
                        transformed_point.point.x,
                        transformed_point.point.y,
                        transformed_point.point.z,
                        self.model.names[int(class_id)]
                    ])

            self.get_logger().info(f"Transformed points written to CSV.")

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

    def __del__(self):
        # Close the file when the node is destroyed
        if self.file:
            self.file.close()

def main(args=None):
    rclpy.init(args=args)
    node = MultiArrayTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
