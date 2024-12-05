import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import numpy as np
import cv2
import time
from tf2_geometry_msgs import do_transform_point
import yaml
from ultralytics import YOLO  
from std_msgs.msg import Float32MultiArray
import os
from ament_index_python.packages import get_package_share_directory

global class_count, class_id_count

yaml_file_path = os.path.join(get_package_share_directory('semantic_mapper'),'config','data.yaml')
with open(yaml_file_path, 'r') as file:
    yaml_content = yaml.safe_load(file)
    class_id_count = {i: 0 for i, name in enumerate(yaml_content['names'])}
    class_count = {name: 0 for name in yaml_content['names']}

class SemanticObject:
    def __init__(self, x_in_odom, y_in_odom, z_in_odom, class_id, x_in_depth = None, y_in_depth = None, z_in_depth = None, marker=None):
        self.obj_id = None
        self.x = x_in_odom
        self.y = y_in_odom
        self.z = z_in_odom
        self.x_in_depth = x_in_depth
        self.y_in_depth = y_in_depth
        self.z_in_depth = z_in_depth
        self.class_id = class_id
        self.marker = marker
        self.timestamp = time.time()
        self.lifespan = 5
        
    @property
    def isAlive(self):
        return time.time() - self.timestamp < self.lifespan
    
    def renew(self):
        self.timestamp = time.time()

class SemanticDB:
    def __init__(self, path2write = None):
        self.objects = []   
        self.same_obj_thres = 0.5
        if path2write is not None:
            self.path2write = path2write
        else:
            self.path2write = "semantic_database.csv"

    def add_object(self, obj: SemanticObject):
        for i in range(len(self.objects)):
            existing_obj = self.objects[i]
            if (
                existing_obj.class_id == obj.class_id
                and abs(existing_obj.x - obj.x) < self.same_obj_thres
                and abs(existing_obj.y - obj.y) < self.same_obj_thres
                and abs(existing_obj.z - obj.z) < self.same_obj_thres
                ):  

                self.objects[i].renew()
                print("Same Object")
                return False
        obj.obj_id = class_id_count[obj.class_id]
        class_id_count[obj.class_id] += 1
        self.objects.append(obj)
        print("New Semantic object discovered of class {}".format(obj.class_id), "Number of objects: {}".format(len(self.objects)))
        return True

    def remove_object(self, obj):
        print(f"Removing object {obj.obj_id} of class {obj.class_id}")
        self.objects.remove(obj)
    
    def write2csv(self):
        with open(self.path2write, 'w') as f:
            f.write("obj_id,class_id,class_name,x,y,z,isAlive\n")
            for obj in self.objects:
                f.write(f"{obj.obj_id},{obj.class_id},{yaml_content['names'][obj.class_id]},{obj.x},{obj.y},{obj.z},{obj.isAlive}\n")

        print(f"Database written to {self.path2write}")

class DepthCamera(Node):
    def __init__(self, debug=False):
        super().__init__('depth_camera_node')

        
        model_path = os.path.join(get_package_share_directory('semantic_mapper'),'config','inter-iit-final4.pt')
        self.model = YOLO(model_path)  
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.bridge = CvBridge()
        self.declare_parameter('bot_name', 'bot_0')
        self.bot_name = self.get_parameter('bot_name').value
        self.source_frame = f'{self.bot_name}/camera_depth_frame'  
        
        # self.target_frame = f'{self.bot_name}/odom'  
        self.target_frame = f'map'  

        self.subscription = self.create_subscription(
            Image,
            f'/{self.bot_name}/camera/image_raw',
            self.image_callback,
            1  
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            f'/{self.bot_name}/camera/depth/camera_info',  
            self.camera_info_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            f'/{self.bot_name}/camera/depth/image_raw',  
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
        self.half_visual_angle = np.pi / 2
        self.conf_thres = 0.7
        self.semantic_db = SemanticDB()

        self.create_timer(1, self.logging_callback)  

    def pixel2depth_transform(self, x, y):
        depth = self.depth_image[y, x]

        x = (x - self.intrinsics['cx']) * depth / self.intrinsics['fx']
        y = (y - self.intrinsics['cy']) * depth / self.intrinsics['fy']
        z = depth

        return x, y, z
    
    def logging_callback(self):
        
        if self.cv_image is None or self.intrinsics is None or self.depth_image is None:
            print("No image received")
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

            if point_stamped_in_odom is None:
                continue    
            x_in_odom = point_stamped_in_odom.point.x
            y_in_odom = point_stamped_in_odom.point.y
            z_in_odom = point_stamped_in_odom.point.z

            if np.isnan(x_in_odom) or np.isnan(y_in_odom) or np.isnan(z_in_odom):
                continue

            semantic_obj = SemanticObject(x_in_odom, y_in_odom, z_in_odom, self.class_id[i], 
                                          x_in_depth, y_in_depth, z_in_depth)
            added = self.semantic_db.add_object(semantic_obj)

            self.check_front_area()

            self.semantic_db.write2csv()

            if self.debug == True:
                cv2.putText(self.annotated_frame, f"({x_in_odom:.2f},{y_in_odom:.2f},{z_in_odom:.2f})", 
                        (self.centroid_x[i], self.centroid_y[i]), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 0, 0), 3, cv2.LINE_AA)
            
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
        removed_objects = []
        front_threshold = 2.0  # Define the front area threshold

        for obj in self.semantic_db.objects:
            # Transform object's position to the robot's frame
            x_in_depth = obj.x_in_depth
            y_in_depth = obj.y_in_depth
            z_in_depth = obj.z_in_depth
            isAlive = obj.isAlive

            # Check if the object is in front of the robot
            if (0 < z_in_depth < front_threshold and 
                abs(x_in_depth) < z_in_depth * np.tan(self.half_visual_angle) and
                not isAlive
                ):
                print(f"Object {obj.obj_id} is Dead.")
                removed_objects.append(obj)

        # Remove objects no longer in the front area
        for obj in removed_objects:
            self.semantic_db.objects.remove(obj)  # Remove from semantic DB
            # self.remove_marker(obj)  # Remove associated marker from RViz
            print(f"Object {obj.obj_id} removed from semantic DB and RViz markers")
        
        

def main(args=None):
    rclpy.init(args=args)
    node = DepthCamera(debug=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
