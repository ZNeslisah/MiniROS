import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class EmergencyLedNode(Node):

    def __init__(self):
        super().__init__('emergency_button')
        self.get_logger().info("Emergency Node has started")

        self.subscription = self.create_subscription(
            Bool,
            'isButtonPressed',
            self.listener_callback,
            10)
        self.publisher_led = self.create_publisher(Bool, 'emergency_led', 10)
        self.publisher_vel = self.create_publisher(Twist, 'emergency_vel', 10)

    def listener_callback(self, msg):
        emergency_led_msg = Bool()
        twist = Twist()
        
        if msg.data:
            self.get_logger().info('Button is pressed, publishing emergency_led = true and zero velocity')
            emergency_led_msg.data = True
            
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_vel.publish(twist)

        else:
            self.get_logger().info('Button is not pressed, turning off emergency_led')
            emergency_led_msg.data = False

        self.publisher_led.publish(emergency_led_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyLedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
