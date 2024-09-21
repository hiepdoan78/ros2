import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

import cv2
import lane_detection.yolov5.detect as detect
import serial
from threading import Lock
class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')
        self.get_logger().info('Looking for the sign...')
        
        self.image_sub = self.create_subscription(
            Image, "/image_i", self.image_callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.stop_publisher = self.create_publisher(Bool, '/stop', 10)

        self.ben_publisher = self.create_publisher(Bool, '/lift', 10)

        self.saved_image_pub = self.create_publisher(Image, "/saved_image", 1)

        self.bridge = CvBridge()

        self.declare_parameter('serial_port', value="/dev/ttyAMA10")
        self.serial_port_name = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=460800)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value

        self.get_logger().info(f"Connecting to port {self.serial_port_name} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0.2)
        self.get_logger().info(f"Connected to {self.conn.port}")
        self.mutex = Lock()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            file = '/ros2/src/lane_detection/lane_detection/yolov5/img.jpg'
            cv2.imwrite(file, cv_image)
            detect.main()
            stopp, lift = detect.main()
           

            self.get_logger().info(f'Status: {lift}')
            if lift == 1:
                self.send_command(f"{'2'}\n")  
            if lift == 0:
                self.send_command(f"{'3'}\n") 
            saved_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.saved_image_pub.publish(saved_image_msg)
            self.stop_publisher.publish(Bool(data=bool(stopp)))
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
    def send_command(self, cmd_string):
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()

def main(args=None):
    rclpy.init(args=args)
    sign_detection_node = SignDetectionNode()
    rclpy.spin(sign_detection_node)
    sign_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()