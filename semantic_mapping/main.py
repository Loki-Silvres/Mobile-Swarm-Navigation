import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO  # Import the YOLO module
from std_msgs.msg import Float32MultiArray

class DepthCamera(Node):
    def __init__(self):
        super().__init__('depth_camera_node')

        # Load YOLO model
        self.model = YOLO('inter-iit1n.pt')  # Replace with your YOLO model path

        # Subscribe to camera info
        self.subscription = self.create_subscription(
            Image,
            '/bot_0/camera1/image_raw',
            self.image_callback,
            1  # QoS
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/bot_0/camera1/depth/camera_info',  # Replace with your topic name
            self.camera_info_callback,
            10
        )
       
        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/bot_0/camera1/depth/image_raw',  # Replace with your topic name
            self.depth_image_callback,
            10
        )

        # Publisher for coordinates
        self.publisher_ = self.create_publisher(Float32MultiArray, '/coordinates_topic', 10)

        self.bridge = CvBridge()
        self.intrinsics = None
        self.annotated_frame = None  # Initialize annotated_frame here
        self.centroid_x = []
        self.centroid_y = []
        self.class_id = []
        self.width = []
    def image_callback(self, msg):
        self.get_logger().info('Received an RGB image!')
        try:
            # Convert ROS Image to OpenCV format
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model(self.cv_image)
            detections = results[0].boxes

            self.centroid_x.clear()  # Clear previous centroids
            self.centroid_y.clear()
            self.class_id.clear()
            self.width.clear()
            if detections is not None and len(detections) > 0:
                for box in detections:
                    # Extract coordinates
                    x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
                    widths = [int(x_max),int(x_min),int(y_min)]   
                    # Calculate centroids
                    centroid_x = (x_min + x_max) / 2
                    centroid_y = (y_min + y_max) / 2
                    self.centroid_x.append(int(centroid_x))
                    self.width.append(widths)
                    self.centroid_y.append(int(centroid_y))
                    self.class_id.append(int(box.cls[0].item()))
                    self.get_logger().info(f"Centroid: ({centroid_x:.2f}, {centroid_y:.2f})")

            # Annotated frame for visualization
            self.annotated_frame = np.array(results[0].plot())  # Annotated image with bounding boxes
            
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")

    def camera_info_callback(self, msg):
        # Extract camera intrinsics from CameraInfo message
        self.intrinsics = {
            'cx': msg.k[2],  # Principal point X
            'cy': msg.k[5],  # Principal point Y
            'fx': msg.k[0],  # Focal length X
            'fy': msg.k[4],  # Focal length Y
        }
        self.get_logger().info(f"Camera intrinsics received: {self.intrinsics}")

    def depth_image_callback(self, msg):
        if self.intrinsics is None:
            self.get_logger().warn("Camera intrinsics not received yet.")
            return
        
        try:
            # Ensure annotated_frame is initialized
            if self.annotated_frame is None:
                self.get_logger().warn("No annotated frame available.")
                return
            
            # Convert ROS Image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Ensure depth image is in the correct format (16-bit or 32-bit)
            if depth_image.dtype == np.uint16:  # Depth in millimeters
                depth_image = depth_image / 1000.0  # Convert to meters
            elif depth_image.dtype == np.float32:  # Depth in meters
                pass  # Depth is already in meters
            else:
                self.get_logger().error("Unsupported depth image format")
                return

            # Process each detected object
            for i in range(len(self.centroid_x)):
                u, v = self.centroid_x[i], self.centroid_y[i]  # Pixel coordinates (e.g., centroids)
                
                depth = depth_image[v, u]
                
                
                x = (u - self.intrinsics['cx']) * depth / self.intrinsics['fx']
                y = (v - self.intrinsics['cy']) * depth / self.intrinsics['fy']
                z = depth


                # Create a Point message to store the 3D coordinates
                msg_coord = Float32MultiArray()
                msg_coord.data = [float(x), float(y),float(z),float(self.class_id[i])]

                # Publish the 3D coordinates to the topic
                self.publisher_.publish(msg_coord)
                self.get_logger().info(f"Publishing 3D coordinates: X={x:.2f}, Y={z:.2f}, Z={y:.2f}")

             

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")
        

        cv2.imshow('Annotated Frame', self.annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DepthCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
