#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelAdjuster(Node):

    def __init__(self):
        super().__init__('cmd_vel_adjuster')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_adjusted', 10)

    def cmd_vel_callback(self, msg):
        if msg.linear.x == 0.0 and msg.angular.z != 0.0:
            msg.linear.x = 0.25
        elif msg.linear.x > 0.5:
            msg.linear.x = 0.5
        elif msg.linear.x < -0.5:
            msg.linear.x = -0.5
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_adjuster = CmdVelAdjuster()
    rclpy.spin(cmd_vel_adjuster)
    cmd_vel_adjuster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
