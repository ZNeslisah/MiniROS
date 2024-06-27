import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import socket
import struct
import threading

MULTICAST_GROUP = '224.0.0.253'
MULTICAST_PORT = 5007

class MulticastListenerNode(Node):

    def __init__(self):
        super().__init__('pipe_read')
        self.get_logger().info("Pipe_read has started")

        self.publisher_ = self.create_publisher(Bool, 'isButtonPressed', 10)
        self.sock = self.create_multicast_socket(MULTICAST_GROUP, MULTICAST_PORT)
        
        # Start listening in a separate thread to avoid blocking
        self.listener_thread = threading.Thread(target=self.listen_to_multicast)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def create_multicast_socket(self, group, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', port))
        mreq = struct.pack("4sl", socket.inet_aton(group), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.setblocking(False)
        return sock

    def listen_to_multicast(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                message = data.decode().strip()
                # self.get_logger().info(f"Received message: {message}")
                message = message.split('|')
                if message[0] == 'isButtonPressed':
                    bool_value = message[1].lower() == '1'
                    self.publish_to_topic(bool_value)
            except OSError as e:
                if e.args[0] == 11:  # EAGAIN, no data available
                    continue
                else:
                    print(f"Error receiving data: {e}")

    def publish_to_topic(self, value):
        msg = Bool()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {value}")

def main(args=None):
    rclpy.init(args=args)
    node = MulticastListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
