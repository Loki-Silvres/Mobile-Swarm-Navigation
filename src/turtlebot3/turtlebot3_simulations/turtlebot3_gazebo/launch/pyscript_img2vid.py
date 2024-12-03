import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv_bridge
import cv2
import numpy as np


class ImageToVideoConverter(Node):
    def __init__(self):
        super().__init__('image_to_video_converter')
        
        # Initialize CvBridge
        self.bridge = cv_bridge.CvBridge()
        
        # Subscribe to the raw image topic
        self.raw_image_sub = self.create_subscription(
            Image,
            'bot_0/camera1/image_raw',
            self.raw_image_callback,
            10
        )
        
        # Publisher for the compressed image topic
        self.compressed_image_pub = self.create_publisher(
            CompressedImage,
            '/camera/image_compressed',
            10
        )
        
        self.video_writer = None

    def raw_image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Compress the image
            _, compressed_image = cv2.imencode('.jpg', cv_image)
            
            # Create and publish a CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = compressed_image.tobytes()
            self.compressed_image_pub.publish(compressed_msg)
            
            # Save the image to video
            self.save_to_video(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def save_to_video(self, image):
        if self.video_writer is None:
            self.init_video_writer(image)
        self.video_writer.write(image)

    def init_video_writer(self, image):
        try:
            height, width, _ = image.shape
            video_filename = 'output_video.mp4'
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            fps = 30  # Adjust FPS as needed
            self.video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
        except Exception as e:
            self.get_logger().error(f"Error initializing video writer: {e}")

    def destroy_node(self):
        if self.video_writer:
            self.video_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    image_to_video_converter = ImageToVideoConverter()
    rclpy.spin(image_to_video_converter)
    image_to_video_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
