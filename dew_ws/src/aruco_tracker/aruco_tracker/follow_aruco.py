import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PolygonStamped
import numpy as np
from rclpy.timer import Timer

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

        self.last_marker_time = self.get_clock().now()
        self.no_marker_timeout = 2.0  # seconds without marker before rotating
        self.timer = self.create_timer(0.1, self.check_marker_timeout)

    def listener_callback(self, msg):
        if msg.polygon.points:
            self.last_marker_time = self.get_clock().now()
            self.get_logger().info(f'Received marker corners {msg.polygon.points}')

            # Calculate the center of the marker
            corners = [(point.x, point.y) for point in msg.polygon.points]
            corners = np.array(corners)
            cX, cY = np.mean(corners, axis=0)

            # Calculate the size of the marker (using the distance between two opposite corners)
            size = np.linalg.norm(corners[0] - corners[2])

            # Image dimensions (assuming some known width)
            w = 1280  # Replace with your camera's width
            self.get_logger().info(f'center: {cX}')

            # Calculate angular velocity based on marker center position
            angular_velocity = -0.00032 * (cX - w / 2)

            # Calculate linear velocity based on marker size
            reference_size = 700  # Size of the marker at the desired distance
            max_linear_velocity = 0.4  # Maximum linear velocity
            min_linear_velocity = 0.05  # Minimum linear velocity

            if size < reference_size:
                linear_velocity = 0.0011 * (reference_size - size)
            else:
                linear_velocity = 0.0
                angular_velocity = 0.0

            # Publish the calculated velocities
            self.publish_velocity(linear_velocity, angular_velocity)
        else:
            self.get_logger().info('No marker detected')

    def publish_velocity(self, linear_velocity, angular_velocity):
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        # Debugging information
        self.get_logger().info(f'Linear velocity: {twist.linear.x}')
        self.get_logger().info(f'Angular velocity: {twist.angular.z}')

        self.publisher_.publish(twist)

    def check_marker_timeout(self):
        current_time = self.get_clock().now()
        time_since_last_marker = (current_time - self.last_marker_time).seconds
        if time_since_last_marker > self.no_marker_timeout:
            self.get_logger().info('No marker detected for a while, rotating to search')
            self.publish_velocity(0.0, 0.38)  # Rotate in place

def main(args=None):
    rclpy.init(args=args)
    follow_aruco = FollowAruco()
    rclpy.spin(follow_aruco)
    follow_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
