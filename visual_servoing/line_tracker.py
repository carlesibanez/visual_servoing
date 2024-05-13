import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point, Twist
from cv_bridge              import CvBridge, CvBridgeError
import visual_servoing.tuning_window as tw
from visual_servoing.process_image import apply_tuning, generate_visualization
import cv2


class LineTracker(Node):
    def __init__(self):
        super().__init__('line_tracker')

        self.get_logger().info('Initializing Line Tracker Node...')
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # self.point_pub = self.create_publisher(Point, '/line/point', 10)
        self.image_out_pub = self.create_publisher(Image, '/line/image_out', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_line', 10)


        # Declare parameters
        self.declare_parameter('tuning_mode', False)

        self.declare_parameter("img_width",640)
        self.declare_parameter("img_height",480)

        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",0)
        self.declare_parameter("y_max",100)

        # HSV parameters
        # Hue: 0-180, Saturation: 0-255, Value: 0-255
        self.declare_parameter("low_h",0)
        self.declare_parameter("high_h",180)
        self.declare_parameter("low_s",0)
        self.declare_parameter("high_s",255)
        self.declare_parameter("low_v",0)
        self.declare_parameter("high_v",255)


        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.window_name = 'Tuning Window'
        self.tuning_params = {
            'img_width': self.get_parameter('img_width').get_parameter_value().integer_value,
            'img_height': self.get_parameter('img_height').get_parameter_value().integer_value,
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'low_h': self.get_parameter('low_h').get_parameter_value().integer_value,
            'high_h': self.get_parameter('high_h').get_parameter_value().integer_value,
            'low_s': self.get_parameter('low_s').get_parameter_value().integer_value,
            'high_s': self.get_parameter('high_s').get_parameter_value().integer_value,
            'low_v': self.get_parameter('low_v').get_parameter_value().integer_value,
            'high_v': self.get_parameter('high_v').get_parameter_value().integer_value,
        }

        self.active_window = [self.tuning_params['x_min'], self.tuning_params['y_min'], 
                              self.tuning_params['x_max'], self.tuning_params['y_max']]

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

        # If using the tuning mode, get the updated tuning parameters
        if self.tuning_mode:
            self.tuning_params = tw.get_tuning_params(self.tuning_params, self.window_name)
            self.active_window = [self.tuning_params['x_min'], self.tuning_params['y_min'], 
                                  self.tuning_params['x_max'], self.tuning_params['y_max']]
            
        # Apply the tuning parameters to the image (preprocessing)
        img_out = apply_tuning(cv_image, self.tuning_params, self.active_window)

        # Find the contours of the line
        conours, hier = cv2.findContours(img_out.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        center = None
        if len(conours) > 0:
            c = max(conours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # Add the offset of the active window
                center = (cx+self.active_window[0], cy+self.active_window[1])


        img_processed = generate_visualization(cv_image, img_out, self.active_window, center, self.tuning_params)
        
        # If tuning mode is enabled, display the tuning window
        if self.tuning_mode:
            cv2.imshow(self.window_name, img_processed)

        # Publish the image with the line drawn on it
        try:
            img_out_msg = self.bridge.cv2_to_imgmsg(img_processed, 'bgr8')
            self.image_out_pub.publish(img_out_msg)
        except CvBridgeError as e:
            print(e)
        
        self.move(center)

    def move(self, center):
        # This function will move the robot based on the center of the line
        # If the line is not found, the robot will start turning counter-clockwise
        # If the line is found, the robot will move forward and turn towards the line

        twist_msg = Twist()
        if center:
            offset = 320 - center[0]
            twist_msg.linear.x = 0.3
            twist_msg.angular.z = offset/100
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.2
        self.vel_pub.publish(twist_msg)

    def convert_rect_perc_to_pixels(self, rect_perc, image):
        # This is a helper function to convert the rectangle coordinates from percentage to pixel values
        rows = image.shape[0]
        cols = image.shape[1]

        scale = [cols, rows, cols, rows]

        return [int(a*b/100) for a,b in zip(rect_perc, scale)]

def main(args=None):
    rclpy.init(args=args)

    line_tracker = LineTracker()

    while rclpy.ok():
        rclpy.spin_once(line_tracker)
        tw.wait_on_gui()

    line_tracker.destroy_node()
    rclpy.shutdown()


