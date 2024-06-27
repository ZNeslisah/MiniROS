#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class CmdVelListener(Node):

    def __init__(self):
        super().__init__('cmd_vel_listener')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Set up the socket connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('144.122.250.48', 4052)  # Replace with your server IP and port
        self.sock.connect(self.server_address)
        self.get_logger().info(f'Connected to socket at {self.server_address}')

    def cmd_vel_callback(self, msg):
        # Extract linear.x and angular.z
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        message = f'({linear_x},{angular_z})'
        self.get_logger().info(f'Received velocity command: {message}')
        
        # Send the message through the socket
        try:
            self.sock.sendall(message.encode())
        except Exception as e:
            self.get_logger().error(f'Failed to send message over socket: {e}')

    def destroy_node(self):
        # Clean up the socket connection
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



