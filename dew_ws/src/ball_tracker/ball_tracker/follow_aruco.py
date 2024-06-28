import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PolygonStamped
import numpy as np

class FollowAruco(Node):
    def __init__(self):
        super().__init__('follow_aruco')
        self.subscription = self.create_subscription(
            PolygonStamped,
            'aruco_marker',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_aruco', 10)
        self.subscription  # prevent unused variable warning
        self.publisher_  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received marker corners {msg.polygon.points}')

        # Calculate the center of the marker
        corners = [(point.x, point.y) for point in msg.polygon.points]
        corners = np.array(corners)
        cX, cY = np.mean(corners, axis=0)

        # Calculate the size of the marker
        size = np.linalg.norm(corners[0] - corners[2])

        # Image dimensions (assuming some known width)
        w = 640  # Replace with your camera's width

        # Calculate angular velocity based on marker center position
        angular_velocity = -0.002 * (cX - w / 2)

        # Calculate linear velocity based on marker size
        # Stop moving forward if the marker is too close (size too big)
        if size > 100:  # You can adjust this threshold
            linear_velocity = 0.0
        else:
            linear_velocity = 0.01 * (100 - size)  # You can adjust the gain

        # Create Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        # Debugging information
        self.get_logger().info(f'Linear velocity: {linear_velocity}')
        self.get_logger().info(f'Angular velocity: {angular_velocity}')

        # Publish velocity command
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    follow_aruco = FollowAruco()
    rclpy.spin(follow_aruco)
    follow_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
