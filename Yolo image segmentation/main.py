import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import matplotlib.pyplot as plt

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.model = YOLO('inter-iit1.pt')  # Replace with the actual path to your YOLO model
        # self.model.to("cuda")  # Use GPU for inference

        # Subscribing to the image topic (replace with the actual topic name)
        self.subscription = self.create_subscription(
            Image,
            '/bot_0/camera1/image_raw',
            self.image_callback,
            10  # QoS
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()  # For converting ROS images to OpenCV


        
    def image_callback(self, msg):
        self.get_logger().info('Received an image!')
        
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run object detection with YOLO
            results = self.model(cv_image)
            annotated_frame = np.array(results[0].plot())  # Annotated frame with bounding boxes and labels
            
            # Display the image using matplotlib (non-blocking)
            plt.imshow(cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB))
            plt.axis('off')  # Turn off axis for a cleaner display
            plt.draw()  # Redraw the image
            plt.pause(0.000000000000000000001)  # Pause briefly to allow for updates

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
