import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import do_transform_point
import csv
from ultralytics import YOLO
import yaml

global class_count, class_id_count

yaml_file_path = '/home/loki/Downloads/data.yaml'
with open(yaml_file_path, 'r') as file:
    yaml_content = yaml.safe_load(file)
    class_id_count = {i: 0 for i, name in enumerate(yaml_content['names'])}
    class_count = {name: 0 for name in yaml_content['names']}

class SemanticObject:
    def __init__(self, x, y, z, class_id, marker=None):
        self.obj_id = None
        self.x = x
        self.y = y
        self.z = z
        self.class_id = class_id
        self.marker = marker
    
class SemanticDB:
    def __init__(self):
        self.objects = []   
        self.same_obj_thres = 1.0

    def add_object(self, obj: SemanticObject):
        for existing_obj in self.objects:
            if (
                existing_obj.class_id == obj.class_id
                and abs(existing_obj.x - obj.x) < self.same_obj_thres
                and abs(existing_obj.y - obj.y) < self.same_obj_thres
                and abs(existing_obj.z - obj.z) < self.same_obj_thres
            ):
                print("Same Object")
                return False
        obj.obj_id = class_id_count[obj.class_id]
        class_id_count[obj.class_id] += 1
        self.objects.append(obj)
        print("New Semantic object discovered of class {}".format(obj.class_id), "Number of objects: {}".format(len(self.objects)))
        # print(class_id_count)
        return True

    def remove_object(self, obj):
        print(f"Removing object {obj.obj_id} of class {obj.class_id}")
        self.objects.remove(obj)


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
        self.model = YOLO('/home/loki/Mobile-Swarm-Navigation/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/launch/inter-iit-final4.pt')  # Replace with your YOLO model path

        # Open the CSV file in append mode
        self.file = open('coordinates.csv', mode='a', newline='')
        self.writer = csv.writer(self.file)
        self.same_obj_thres = 0.5
        self.semantic_db = SemanticDB()

        self.create_timer(0.1, self.check_front_area)

        self.colors = {  # Add colors for 18 classes
            0: (1.0, 0.0, 0.0, 1.0),   # Red
            1: (0.0, 1.0, 0.0, 1.0),   # Green
            2: (0.0, 0.0, 1.0, 1.0),   # Blue
            3: (1.0, 1.0, 0.0, 1.0),   # Yellow
            4: (1.0, 0.0, 1.0, 1.0),   # Magenta
            5: (0.0, 1.0, 1.0, 1.0),   # Cyan
            6: (0.5, 0.0, 0.5, 1.0),   # Purple
            7: (0.5, 0.5, 0.0, 1.0),   # Olive
            8: (0.0, 0.5, 0.5, 1.0),   # Teal
            9: (0.5, 0.5, 0.5, 1.0),   # Gray
            10: (1.0, 0.5, 0.0, 1.0),  # Orange
            11: (0.0, 1.0, 0.5, 1.0),  # Spring Green
            12: (0.5, 0.0, 1.0, 1.0),  # Violet
            13: (1.0, 0.0, 0.5, 1.0),  # Rose
            14: (0.5, 1.0, 0.0, 1.0),  # Lime
            15: (0.0, 0.5, 1.0, 1.0),  # Sky Blue
            16: (0.5, 0.5, 1.0, 1.0),  # Light Purple
            17: (1.0, 0.5, 0.5, 1.0),  # Salmon
        }
        
    def multiarray_callback(self, msg):
        # Extract data from Float32MultiArray
        points = msg.data  # This is a flattened array of [x, y, z, class_id, ...]

        for i in range(0, len(points), 4):  # Assuming every 4th element is the class_id
            x, y, z, class_id = points[i], points[i+1], points[i+2], points[i+3]
            
            if z > 4:
                continue
            # Transform the point
            transformed_point = self.transform_point(x, y, z)
            
            if transformed_point:
                transformed_x = transformed_point.point.x
                transformed_y = transformed_point.point.y
                transformed_z = transformed_point.point.z
                semantic_object = SemanticObject(transformed_x, transformed_y, transformed_z, class_id)

                added = self.semantic_db.add_object(semantic_object)
                if not added:
                    continue
                # Write the transformed point and class_id to CSV
                self.writer.writerow([
                    transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z,
                    self.model.names[int(class_id)]
                ])
                # Publish a marker to RViz
                self.publish_marker(transformed_point, class_id)

            # self.get_logger().info(f"Transformed points written to CSV.")

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

        # Set marker color based on class ID
        color = self.colors.get(class_id, (1.0, 1.0, 1.0, 1.0))  # Default to white if class_id not found
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        # Publish the marker
        self.marker_publisher.publish(marker)

    def check_front_area(self):
        # Define the area in front of the robot to check
        front_threshold = 1.0  # Distance threshold in meters
        removed_objects = []

        for obj in self.semantic_db.objects:
            # Check if the object is in front of the robot
            if obj.x < front_threshold and abs(obj.y) < front_threshold:
                print(f"Object {obj.obj_id} is still in front")
            else:
                removed_objects.append(obj)

        # Remove objects no longer in the front area
        for obj in removed_objects:
            self.semantic_db.remove_object(obj)
            self.remove_marker(obj)

    def timer_callback(self):
        removed_objects = []
        front_threshold = 1.0  # Define the front area threshold

        for obj in self.semantic_db.objects:
            # Transform object's position to the robot's frame
            point_in_odom = PointStamped()
            point_in_odom.header.frame_id = self.target_frame  # 'odom' frame
            point_in_odom.header.stamp = self.get_clock().now().to_msg()
            point_in_odom.point.x = obj.x
            point_in_odom.point.y = obj.y
            point_in_odom.point.z = obj.z

            try:
                # Transform to the robot's base frame
                transform = self.tf_buffer.lookup_transform(
                    self.source_frame,  # Robot's base frame
                    self.target_frame,  # 'odom' frame
                    rclpy.time.Time(),
                    timeout=rclpy.time.Duration(seconds=1.0)
                )
                point_in_robot_frame = do_transform_point(point_in_odom, transform)

                # Check if the object is in front of the robot
                if (0 < point_in_robot_frame.point.x < front_threshold and 
                    abs(point_in_robot_frame.point.y) < front_threshold):
                    print(f"Object {obj.obj_id} is still in front")
                else:
                    removed_objects.append(obj)

            except Exception as e:
                self.get_logger().error(f"Error transforming object {obj.obj_id}: {e}")
                continue

        # Remove objects no longer in the front area
        for obj in removed_objects:
            self.semantic_db.objects.remove(obj)  # Remove from semantic DB
            self.remove_marker(obj)  # Remove associated marker from RViz
            print(f"Object {obj.obj_id} removed from semantic DB and RViz markers")

    def remove_marker(self, obj):
        # Send a marker DELETE action for the removed object
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "transformed_points"
        marker.id = obj.obj_id
        marker.action = Marker.DELETE
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
