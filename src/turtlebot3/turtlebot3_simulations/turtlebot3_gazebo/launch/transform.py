import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
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
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.source_frame = 'bot_0/camera_rgb_depth_frame'  # Replace with your source frame
        self.target_frame = 'bot_0/odom'  # Replace with your target frame
        self.model = YOLO('/home/loki/Mobile-Swarm-Navigation/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/launch/inter-iit1n.pt')  # Replace with your YOLO model path

        # Open the CSV file in append mode
        self.file = open('coordinates.csv', mode='a', newline='')
        self.writer = csv.writer(self.file)
        
    def multiarray_callback(self, msg):
        # Extract data from Float32MultiArray
        points = msg.data  # This is a flattened array of [x, y, z, class_id, ...]

        for i in range(0, len(points), 4):  # Assuming every 4th element is the class_id
            x, y, z, class_id = points[i], points[i+1], points[i+2], points[i+3]
            if z < 4:
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
                    # Publish a marker to RViz
                    self.publish_marker(transformed_point, class_id)

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

    def publish_marker(self, transformed_point, class_id):
        # Create a marker message
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "transformed_points"
        marker.id = int(class_id)
        marker.type = Marker.SPHERE  # Use a sphere to represent points
        marker.action = Marker.ADD
        marker.pose.position.x = transformed_point.point.x
        marker.pose.position.y = transformed_point.point.y
        marker.pose.position.z = transformed_point.point.z
        marker.pose.orientation.w = 1.0

        # Customize marker appearance
        marker.scale.x = 0.2  # Sphere radius
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Color the marker (RGBA)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Publish the marker
        self.marker_publisher.publish(marker)

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
