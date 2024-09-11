import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge, CvBridgeError
import cv2
import lane_detection.process_image as proc

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.get_logger().info('Looking for the lane...')
        self.image_sub = self.create_subscription(Image,"/image_in",self.image_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.result_pub = self.create_publisher(Image, "/image_out", 1)
        self.lane_pub = self.create_publisher(Point, "/detected_lanes", 1)

        self.bridge = CvBridge()
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # combine denoise frame and gaussian blur to remove noise from the frames
            blur = proc.denoise_frame(cv_image)


            # perspective tranform
            bird_eye_view = proc.warp_perspective(blur)

            # change color from RGB to Binary
            binary_image3 = proc.binary(bird_eye_view)


            # find contour
            lines_3 = proc.detect_line(binary_image3)

            # draw line
            result3 = proc.draw(bird_eye_view, lines_3)

            #result3 = offset(bird_eye_view, lines_3)
            result4 = proc.find_offset(bird_eye_view, lines_3)

            
            result_image_msg = self.bridge.cv2_to_imgmsg(result3, "bgr8")
            result_image_msg = self.bridge.cv2_to_imgmsg(result4, "bgr8")

            self.result_pub.publish(result_image_msg)

            
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = LaneDetectionNode()
    rclpy.spin(lane_detection_node)
    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()