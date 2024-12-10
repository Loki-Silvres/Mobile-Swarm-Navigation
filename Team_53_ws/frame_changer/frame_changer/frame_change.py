import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy,QoSHistoryPolicy

class FrameNameChangerNode(Node):
    def __init__(self):
        super().__init__('frame_name_changer')
        tf_static_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.declare_parameter('ns_name', 'bot_0') 
        self.ns_name = self.get_parameter('ns_name').get_parameter_value().string_value
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.tf_subscriber = self.create_subscription(
            TFMessage,
            f'/{self.ns_name}/tf',  # Subscribe to the /ns/tf topic
            self.tf_callback,
            10
        )
        self.tf_static_subscriber = self.create_subscription(
            TFMessage,
            f'/{self.ns_name}/tf_static',  # Subscribe to the /ns_name/tf_static topic
            self.tf_static_callback,
            10
        )

        self.tf_static_publisher = self.create_publisher(TFMessage, '/tf_static', tf_static_qos)

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map':
                    
                transform.child_frame_id = f'{self.ns_name}/{transform.child_frame_id}'
            else:

                transform.header.frame_id = f'{self.ns_name}/{transform.header.frame_id}'
                transform.child_frame_id = f'{self.ns_name}/{transform.child_frame_id}'

            self.tf_broadcaster.sendTransform(transform)

    def tf_static_callback(self, msg: TFMessage):
        static_transforms = []
        for transform in msg.transforms:
            transform.header.frame_id = f'{self.ns_name}/{transform.header.frame_id}'
            transform.child_frame_id = f'{self.ns_name}/{transform.child_frame_id}'

            static_transforms.append(transform)
        
        self.static_tf_broadcaster.sendTransform(static_transforms)
        # self.tf_static_publisher.publish(static_transforms)


def main(args=None):
    rclpy.init(args=args)

    node = FrameNameChangerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

