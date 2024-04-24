import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
import visual_servoing.tuning_window as tw
import cv2


class LineTracker(Node):
    def __init__(self):
        super().__init__('line_tracker')

        self.get_logger().info('Initializing Line Tracker Node...')
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # self.point_pub = self.create_publisher(Point, '/line/point', 10)
        self.image_out_pub = self.create_publisher(Image, '/line/image_out', 10)


        # Declare parameters
        self.declare_parameter('tuning_mode', True)

        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",0)
        self.declare_parameter("y_max",100)

        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
        }

        self.bridge = CvBridge()

        if self.tuning_mode:
            self.get_logger().info('Tuning mode is enabled')
            tw.tuning_window(self.tuning_params)
            

    def image_callback(self, msg):
        # Retrieve the image from the message
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)

        # TODO: Implement line tracking algorithm here

        # If tuning mode is enabled, display the tuning window
        if self.tuning_mode:
            self.tuning_params = tw.get_tuning_params(self.tuning_params)
            img_out = tw.tuned_image(cv_image, self.tuning_params)
            cv2.imshow('Tuning Window', img_out)

        # Publish the image with the line drawn on it
        try:
            img_out_msg = self.bridge.cv2_to_imgmsg(img_out, 'bgr8')
            self.image_out_pub.publish(img_out_msg)
        except CvBridgeError as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)

    line_tracker = LineTracker()

    while rclpy.ok():
        rclpy.spin_once(line_tracker)
        tw.wait_on_gui()

    line_tracker.destroy_node()
    rclpy.shutdown()


