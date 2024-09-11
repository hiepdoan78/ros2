#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Joy
import serial
from threading import Lock

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_controller')
        

        self.ben_publisher = self.create_publisher(Bool, '/lift', 10)
        
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

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

        self.ben_state = False
        self.last_button_state = 0
   
    def joy_callback(self, msg):
        current_button_state = msg.buttons[4]
        if current_button_state == 1 and self.last_button_state == 0:  
            self.ben_state = not self.ben_state  
            self.send_command(f"{'2' if self.ben_state else '3'}\n")  
            self.get_logger().info(f"Truck lift: {'On' if self.ben_state else 'Off'}")  
            self.ben_publisher.publish(Bool(data=self.ben_state))  

        self.last_button_state = current_button_state  
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
    serial_controller = SerialController()
    rclpy.spin(serial_controller)
    serial_controller.destroy_node()
    serial_controller.close_conn()
    rclpy.shutdown()

if __name__ == '__main__':
    main()