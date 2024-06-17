import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_interfaces.srv import GetRgbDepth
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

class DepthCamNode(Node):

    def __init__(self):
        super().__init__('depth_cam_node')
        self.bridge = CvBridge()

        # RGB와 Depth 이미지 구독 설정
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.rgb_callback,
            qos_profile_sensor_data)
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_profile_sensor_data)

        self.rgb_image = None
        self.depth_image = None

        self.srv = self.create_service(GetRgbDepth, 'get_rgb_depth', self.get_rgb_depth_callback)

    def rgb_callback(self, msg):
        self.rgb_image = msg

    def depth_callback(self, msg):
        self.depth_image = msg

    def get_rgb_depth_callback(self, request, response):
        if self.rgb_image is not None and self.depth_image is not None:
            response.rgb = self.rgb_image
            response.depth = self.depth_image
            self.get_logger().info('Sent RGB and Depth images')
        else:
            self.get_logger().warn('No images available to send')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DepthCamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
