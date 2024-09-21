import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool  # Import Bool
import cv2
import lane_detection.yolov5.process_image as proc
from math import tan
class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.get_logger().info('Looking for the lane...')
        
        self.image_sub = self.create_subscription(
            Image, "/image_in", self.image_callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        
        self.result_pub = self.create_publisher(Image, "/image_out", 1)
        self.lane_pub = self.create_publisher(Point, "/detected_lanes", 1)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.saved_image_pub = self.create_publisher(Image, "/saved_image", 1)

        self.bridge = CvBridge()
        self.steering_angle = 0.0  
        self.vel = 0.1
        self.stop = False 

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.stop_subscription = self.create_subscription(
            Bool,
            '/stop',
            self.stop_callback,
            10)
        
    def stop_callback(self, msg):
        self.stop = msg.data  
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            blur = proc.denoise_frame(cv_image)
            bird_eye_view = proc.warp_perspective(blur)
            binary_image3 = proc.binary(bird_eye_view)
            lines_3 = proc.detect_line(binary_image3)
            result3 = proc.draw(bird_eye_view, lines_3)
            result4 = proc.find_offset(bird_eye_view, lines_3)
            self.steering_angle = proc.steering_angle(bird_eye_view, lines_3)
            
            result_image_msg = self.bridge.cv2_to_imgmsg(result3, "bgr8")
            self.result_pub.publish(result_image_msg)
            saved_image_msg = self.bridge.cv2_to_imgmsg(blur, "bgr8")
            self.saved_image_pub.publish(saved_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.vel  
        # self.get_logger().info(f'Stop: {self.stop}')
        wheelbase = 0.26 
        if self.stop == 0:
            self.vel = 0.1
            msg.angular.z = 2.0 * (tan(self.steering_angle)) / wheelbase
        if self.stop == 1:
            self.vel = 0.0
            msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()
    rclpy.spin(lane_detection_node)
    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()