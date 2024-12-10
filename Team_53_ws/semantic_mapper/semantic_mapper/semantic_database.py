# !/usr/bin/env python3

from ament_index_python import get_package_share_directory
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
import numpy as np
import cv2
import time
from tf2_geometry_msgs import do_transform_point
import yaml
import os
from ultralytics import YOLO  

global class_count, class_id_count

yaml_file_path = yaml_file_path = os.path.join(get_package_share_directory('semantic_mapper'),'config','data.yaml')
with open(yaml_file_path, 'r') as file:
    yaml_content = yaml.safe_load(file)
    class_id_count = {i: 0 for i, name in enumerate(yaml_content['names'])}
    class_count = {name: 0 for name in yaml_content['names']}

class SemanticObject:
    def __init__(self, x_in_odom, y_in_odom, z_in_odom, class_id, 
                 x_in_depth = None, y_in_depth = None, z_in_depth = None, target_frame = 'map', marker=None):
        """
        Constructor for SemanticObject. 

        Parameters:
        x_in_odom (float): x coordinate in odometry frame
        y_in_odom (float): y coordinate in odometry frame
        z_in_odom (float): z coordinate in odometry frame
        class_id (int): class id of object
        x_in_depth (float): x coordinate in depth frame (optional)
        y_in_depth (float): y coordinate in depth frame (optional)
        z_in_depth (float): z coordinate in depth frame (optional)
        target_frame (str): frame name of target frame (default: 'map')
        marker (Marker): marker message (optional)
        """
        
        self.obj_id = None
        self.x = x_in_odom
        self.y = y_in_odom
        self.z = z_in_odom
        self.x_in_depth = x_in_depth
        self.y_in_depth = y_in_depth
        self.z_in_depth = z_in_depth
        self.class_id = class_id
        self.marker = marker
        self.target_frame = target_frame
        self.timestamp = time.time()
        self.lifespan = 5
        self.colors = {                # Colors for 18 classes
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
        
    @property
    def isAlive(self):
        """
        Check if the SemanticObject is still alive based on its lifespan.

        Returns:
            bool: True if the object is alive (i.e., the current time minus the timestamp 
            is less than the lifespan), otherwise False.
        """
        return time.time() - self.timestamp < self.lifespan
    
    def renew(self):
        self.timestamp = time.time()

    def addMarker(self):
        """
        Create a marker for the SemanticObject and store it in the object's 'marker' attribute.

        The marker is a sphere with a radius of 0.2 m, positioned at the object's coordinates in the 'target_frame'.
        The color of the marker is determined by the object's class id, with default color being white if the class id is not found.
        The marker is added to the ROS topic "transformed_points" with action Marker.ADD.
        """
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = SemanticMapper().get_clock().now().to_msg()
        marker.ns = "transformed_points"
        marker.id = int(self.class_id * 10000 + self.obj_id)
        print(f'Marker id: {marker.id} added')
        marker.type = Marker.SPHERE  
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2 
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        color = self.colors.get(self.class_id, (1.0, 1.0, 1.0, 1.0))  # Default to white if class_id not found
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        self.marker = marker

class SemanticDB:
    def __init__(self, path2write = None):
        
        self.objects = []   
        self.same_obj_thres = 0.5
        self.named_window = "Semantic Database"
        if path2write is not None:
            self.path2write = path2write
        else:
            self.path2write = os.path.join(get_package_share_directory('semantic_mapper'),'map','semantic_database.csv')

            # self.path2write = '/home/loki/semantic_database.csv'

    def add_object(self, obj: SemanticObject):
        """
        Add a SemanticObject to the database.

        If the object is already in the database (i.e. same class_id and position within same_obj_thres), 
        the existing object is renewed and the function returns False. 
        Otherwise, the object is added to the database with a new obj_id, and the function returns True.

        :param obj: The SemanticObject to be added to the database
        :type obj: SemanticObject
        :return: True if the object is new, False if it is already in the database
        :rtype: bool
        """
        for i in range(len(self.objects)):
            existing_obj = self.objects[i]
            if (
                existing_obj.class_id == obj.class_id
                and abs(existing_obj.x - obj.x) < self.same_obj_thres
                and abs(existing_obj.y - obj.y) < self.same_obj_thres
                ):  

                self.objects[i].renew()
                print("Same Object")
                return False
        obj.obj_id = class_id_count[obj.class_id]
        class_id_count[obj.class_id] += 1
        obj.addMarker()
        self.objects.append(obj)
        print("New Semantic object discovered of class {}".format(obj.class_id), "Number of objects: {}".format(len(self.objects)))
        return True

    def remove_object(self, obj):
        print(f"Removing object {obj.obj_id} of class {obj.class_id}")
        self.objects.remove(obj)
    
    def write2csv(self):
        with open(self.path2write, 'w') as f:
            f.write("obj_id,class_id,class_name,x,y,z,isAlive,x_depth,y_depth,z_depth\n")
            var = False
            for obj in self.objects:
                var = f.write(f"{obj.obj_id},{obj.class_id},{yaml_content['names'][obj.class_id]},{obj.x},{obj.y},{obj.z},{obj.isAlive},{obj.x_in_depth},{obj.y_in_depth},{obj.z_in_depth}\n")
            f.close()
        print(f"Database written to {self.path2write} successfully: {var}")

class SemanticMapper(Node):
    def __init__(self, debug=False):
        """
        Initialize the SemanticMapper node.

        This constructor sets up the YOLO model, transformation listeners, 
        publishers, and subscriptions needed for the semantic mapping process. 
        It initializes parameters for the robot, subscribes to image streams, 
        and sets up data structures for processing image and depth information.

        Parameters:
        debug (bool): A flag indicating whether to run the node in debug mode. 
                    Default is False.

        Attributes:
        model_path (str): The model weights path for object detection.
        tf_buffer (Buffer): A buffer for storing transformations.
        tf_listener (TransformListener): A listener for transformations.
        marker_publisher (Publisher): A publisher for visualization markers.
        bridge (CvBridge): A CvBridge for converting ROS images to OpenCV images.
        bot_name (str): The name of the robot.
        source_frame (str): The source frame for the camera.
        target_frame (str): The target frame for transformations.
        subscription (Subscription): Subscription to the RGB image topic.
        camera_info_sub (Subscription): Subscription to the camera info topic.
        depth_sub (Subscription): Subscription to the depth image topic.
        intrinsics (Optional): Camera intrinsics.
        annotated_frame (Optional): Annotated frame for visualization.
        cv_image (Optional): The current image in OpenCV format.
        depth_image (Optional): The current depth image.
        centroid_x (list): List of centroid x-coordinates.
        centroid_y (list): List of centroid y-coordinates.
        class_id (list): List of class IDs detected.
        debug (bool): Debug mode flag.
        half_visual_angle (float): Half of the visual angle.
        conf_thres (float): Confidence threshold for detections.
        register_obj_thres_z (float): Threshold for registering objects based on depth.
        semantic_db (SemanticDB): The database for storing semantic objects.
        """
        super().__init__('semantic_mapper_node')

        model_path = os.path.join(get_package_share_directory('semantic_mapper'),'weights','best.pt')
        self.model = YOLO(model_path)  
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)

        self.bridge = CvBridge()
        self.declare_parameter('bot_name', 'bot_0')
        self.bot_name = self.get_parameter('bot_name').value
        self.source_frame = f'{self.bot_name}/camera_depth_frame'  
        self.source_frame = f'{self.bot_name}/camera_depth_frame'  
        self.target_frame = f'map'  

        self.subscription = self.create_subscription(
            Image,
            f'{self.bot_name}/camera/image_raw',
            self.image_callback,
            1  
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            f'{self.bot_name}/camera/depth/camera_info',  
            self.camera_info_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            f'{self.bot_name}/camera/depth/image_raw',  
            self.depth_image_callback,
            10
        )
        self.intrinsics = None
        self.annotated_frame = None  
        self.cv_image = None
        self.depth_image = None
        
        self.centroid_x = []
        self.centroid_y = []
        self.class_id = []
        self.width = []
        self.debug = debug
        self.half_visual_angle = np.pi / 4
        self.conf_thres = 0.7
        self.register_obj_thres_z = 4.0
        self.semantic_db = SemanticDB()

        self.create_timer(1, self.logging_callback)  

    def pixel2depth_transform(self, x, y):
        """
        Convert a pixel coordinate in the image frame to a point in the camera frame, given the depth image.

        :param x: The x-coordinate of the pixel in the image frame.
        :param y: The y-coordinate of the pixel in the image frame.
        :return: The 3D point in the camera frame.
        """
        depth = self.depth_image[y, x]

        x = (x - self.intrinsics['cx']) * depth / self.intrinsics['fx']
        y = (y - self.intrinsics['cy']) * depth / self.intrinsics['fy']
        z = depth

        return x, y, z
    
    def logging_callback(self):
        
        """
        A callback function that processes images and depth information from the camera.

        This function is called at a regular interval (0.5 seconds) and processes the latest RGB image and depth information
        from the camera. It runs the YOLO model on the image and extracts the bounding boxes of detected objects. Then, 
        it converts the bounding box coordinates from the image frame to the camera frame using the depth information. 
        Finally, it adds the detected objects to the semantic database and visualizes the annotated image.

        """
        if self.cv_image is None or self.intrinsics is None or self.depth_image is None:
            return
        results = self.model(self.cv_image)
        detections = results[0].boxes

        self.centroid_x.clear()  
        self.centroid_y.clear()
        self.class_id.clear()
        self.width.clear()

        if detections is not None and len(detections) > 0:
            for box in detections:
                
                conf = box.conf[0].item()
                if conf < self.conf_thres:
                    continue
                x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
                widths = [int(x_max),int(x_min),int(y_min)] 
                
                centroid_x = (x_min + x_max) / 2
                centroid_y = (y_min + y_max) / 2
                self.centroid_x.append(int(centroid_x))
                self.centroid_y.append(int(centroid_y))
                self.class_id.append(int(box.cls[0].item()))
                self.width.append(widths)
                self.get_logger().info(f"Class: {self.model.names[int(box.cls[0].item())]}, Centroid: ({centroid_x:.2f}, {centroid_y:.2f})")

        
        self.annotated_frame = np.array(results[0].plot())  

        for i in range(len(self.centroid_x)):
            
            x_in_depth, y_in_depth, z_in_depth = self.pixel2depth_transform(self.centroid_x[i], self.centroid_y[i])
            point_stamped_in_odom = self.depth2odom_transform(float(x_in_depth), float(y_in_depth), float(z_in_depth))

            if point_stamped_in_odom is None or z_in_depth > self.register_obj_thres_z:
                continue    
            x_in_odom = point_stamped_in_odom.point.x
            y_in_odom = point_stamped_in_odom.point.y
            z_in_odom = point_stamped_in_odom.point.z

            if np.isnan(x_in_odom) or np.isnan(y_in_odom) or np.isnan(z_in_odom):
                continue

            semantic_obj = SemanticObject(x_in_odom, y_in_odom, z_in_odom, self.class_id[i], 
                                          x_in_depth, y_in_depth, z_in_depth, self.target_frame)
            added = self.semantic_db.add_object(semantic_obj)


            if self.debug == True:
                cv2.putText(self.annotated_frame, f"({x_in_odom:.2f},{y_in_odom:.2f},{z_in_odom:.2f})", 
                        (self.centroid_x[i], self.centroid_y[i]), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 0, 0), 3, cv2.LINE_AA)
                
        self.check_front_area()
        self.semantic_db.write2csv()

        for obj in self.semantic_db.objects:
            self.marker_publisher.publish(obj.marker)
            
        if self.debug == True:
            resized_image = cv2.resize(self.annotated_frame, dsize=(1000, 1000), interpolation=cv2.INTER_LINEAR)
            cv2.imshow('Annotated Frame', resized_image)
            cv2.waitKey(1)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")

    def camera_info_callback(self, msg):
        
        self.intrinsics = {
            'cx': msg.k[2],  
            'cy': msg.k[5],  
            'fx': msg.k[0],  
            'fy': msg.k[4],  
        }
        
    def depth_image_callback(self, msg):
        if self.intrinsics is None:
            self.get_logger().warn("Camera intrinsics not received yet.")
            return
        
        try:           
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if self.depth_image.dtype == np.uint16:  
                self.depth_image = self.depth_image / 1000.0  
            elif self.depth_image.dtype == np.float32:  
                pass  
            else:
                self.get_logger().error("Unsupported depth image format")
                return

            if self.annotated_frame is None:
                self.get_logger().warn("No annotated frame available.")
                return
            
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def depth2odom_transform(self, x, y, z, source_frame = None, target_frame= None):
        try:
            
            point = PointStamped()
            point.header.frame_id = self.source_frame
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = x
            point.point.y = y
            point.point.z = z

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
        
    def check_front_area(self):
        """
        Check if objects in semantic database are in front of the robot and remove them if not alive.

        This function transforms the positions of all objects in the semantic database to the robot's base frame and checks
        if they are in front of the robot (i.e., within a certain distance and visual angle of the robot). If an object is
        not alive and not in front of the robot, it is removed from the semantic database and its associated marker is
        removed from RViz.

        :return: None
        """
        removed_objects = []
        front_threshold = 3.0

        for obj in self.semantic_db.objects:
            isAlive = obj.isAlive
            point_in_odom = PointStamped()
            point_in_odom.header.frame_id = self.target_frame  # 'odom' frame
            point_in_odom.header.stamp = self.get_clock().now().to_msg()
            point_in_odom.point.x = obj.x
            point_in_odom.point.y = obj.y
            point_in_odom.point.z = obj.z

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.source_frame,  # Robot's depth camera frame
                    self.target_frame,  # 'odom' frame
                    rclpy.time.Time(),
                    timeout=rclpy.time.Duration(seconds=1.0)
                )
                point_in_robot_frame = do_transform_point(point_in_odom, transform)
            except Exception as e:
                self.get_logger().error(f"Error transforming object {obj.obj_id}: {e}")
                continue

            # Check if the object is in front of the robot
            if (0 < point_in_robot_frame.point.z < front_threshold and 
                abs(point_in_robot_frame.point.x) < point_in_robot_frame.point.z / np.tan(self.half_visual_angle) and
                not isAlive
                ):
                print(f"Object {obj.obj_id} is Dead.")
                removed_objects.append(obj)

        for obj in removed_objects:
            self.semantic_db.objects.remove(obj)  # Remove from semantic DB
            self.remove_marker(obj)  # Remove associated marker from RViz
            print(f"Object {obj.obj_id} removed from semantic DB and RViz markers")

    def remove_marker(self, obj):
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "transformed_points"
        marker.id = int(obj.class_id * 10000 + obj.obj_id)
        print(f'Marker id: {marker.id} removed')
        marker.action = Marker.DELETE
        self.marker_publisher.publish(marker)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapper(debug=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
