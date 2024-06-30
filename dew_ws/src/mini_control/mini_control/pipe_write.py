import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
from geometry_msgs.msg import Twist

class PicoSender(Node):

    def __init__(self):
        super().__init__('pico_sender')
        self.get_logger().info("PicoSender has started")
        # Subscriptions
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',  # Replace with your ROS2 topic name
            self.cmd_vel_callback,
            10)

        self.subscription_pixel_color = self.create_subscription(
            String,
            'pixel_color',  # Replace with your ROS2 topic name
            self.pixel_color_callback,
            10)
        
        self.subscription_battery_brightness = self.create_subscription(
            String,
            'battery_color',  # New topic subscription
            self.battery_brightness_callback,
            10)
        
        self.subscription_sobe = self.create_subscription(
            Bool,
            'sobe',  # New topic subscription
            self.sobe_callback,
            10)
        
        
        # UDP socket setup
        self.udp_ip = "224.0.0.253"  # Multicast IP
        self.udp_port = 5007         # Multicast port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        
        # Set the TTL (time-to-live) for the multicast message to 1 (local network)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        message = f'twoWheel|{linear_x},{angular_z}'
    
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
        # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')
    
    def pixel_color_callback(self, msg):
        message = f'neopixel|{msg.data}'
    
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
        # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')
    
    def battery_brightness_callback(self, msg):
        message = f'battery|{msg.data}'
    
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
        # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')
    def sobe_callback(self, msg):
        message = f'sobe|{msg.data}'
    
        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
        # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')    

def main(args=None):
    rclpy.init(args=args)
    pico_sender = PicoSender()
    rclpy.spin(pico_sender)
    pico_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






































# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import socket
# from geometry_msgs.msg import Twist

# class PicoSender(Node):

#     def __init__(self):
#         super().__init__('pico_sender')
#         self.get_logger().info("Pipe_write has started")
#         # Subscriptions
#         self.subscription_cmd_vel = self.create_subscription(
#             Twist,
#             '/cmd_vel',  # Replace with your ROS2 topic name
#             self.cmd_vel_callback,
#             10)

#         self.subscription_pixel_color = self.create_subscription(
#             String,
#             'pixel_color',  # Replace with your ROS2 topic name
#             self.pixel_color_callback,
#             10)
        
#         # UDP socket setup
#         self.udp_ip = "224.0.0.253"  # Multicast IP
#         self.udp_port = 5007       # Multicast port
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        
#         # Set the TTL (time-to-live) for the multicast message to 1 (local network)
#         self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    
#     def cmd_vel_callback(self, msg):
#         linear_x = msg.linear.x
#         angular_z = msg.angular.z
#         message = f'keyboard|{linear_x},{angular_z}'
    
#         self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
#         # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')
    
#     def pixel_color_callback(self, msg):
#         message = f'neopixel|{msg.data}'
    
#         self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
#         # self.get_logger().info(f'Sending message: "{message}" to {self.udp_ip}:{self.udp_port}')

# def main(args=None):
#     rclpy.init(args=args)
#     udp_sender = PicoSender()
#     rclpy.spin(udp_sender)
#     udp_sender.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
