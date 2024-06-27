#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys
import select
import termios
import tty
from threading import Thread

class NeslisahTeleopKey(Node):
    def __init__(self):
        super().__init__('neslisah_teleop_key')

        # Flag to indicate if an obstacle is detected
        self.obstacle_detected = False
        self.previous_message = False

        self.get_logger().info("Initializing the NeslisahTeleopKey class...")
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.continuous_control = False  # Continuous Command Publication
        self.pause_motion = True
        self.mode = 'pause'
        self.ispressed = False
        self.exit = False
        self.keySettings = termios.tcgetattr(sys.stdin)

        # Subscribers and Publishers
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.pause_pub = self.create_publisher(Bool, '/pause_motion', 10)

        # Load parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('cmd_rate', 10)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.cmd_rate = self.get_parameter('cmd_rate').value

        self.key_thread = Thread(target=self.key_input, daemon=True)
        self.key_thread.start()

        self.create_timer(1.0 / self.cmd_rate, self.publish_commands)

    def print_instruction(self):
        """
        Print Control Instructions
        """
        print('***********************************************************')
        print('Improved version of keyboard control by Neslisah')
        print('     w            w/x : increase/descrease linear speed')
        print('  a  s  d         a/d : increase/decrease angular speed')
        print('     x            space key, s : force stop')
        print('                  p/m: pause/move')
        print('                  c/n: continous/noncontinuous\n')
        print('Press <ctrl-c> or <q> to exit')
        print('***********************************************************')

    def print_settings(self):
        """
        Print current teleoperation settings
        """
        print('Current Mode: {}, Continuous: {}, Linear Speed: {:.2f}, Angular Speed: {:.2f}'.format(
            self.mode, self.continuous_control, self.linear_speed, self.angular_speed))

    def getkey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.keySettings)
        return key

    def obstacle_callback(self, msg):
        # Called when a new message is received on 'obstacle_detected' topic.
        if msg.data and not self.previous_message:
            self.get_logger().warn("Obstacle detected! Resetting teleop...")
            self.reset_controls()  # Reset the teleop controls
            self.previous_message = True
        elif not msg.data:
            self.previous_message = False

    def key_input(self):
        self.get_logger().info("Initializing key_input")

        self.print_instruction()
        self.print_settings()

        kInput = 0
        while rclpy.ok() and not self.exit:
            key = self.getkey()
            ischanged = False  # Flag variable if the steering input is changed
            if key == 'w':
                self.linear_speed = self.linear_speed + 0.1 * self.max_linear_speed
                self.linear_speed = min(self.linear_speed, self.max_linear_speed)
                self.ispressed = True
                ischanged = True
            if key == 'x':
                self.linear_speed = self.linear_speed - 0.1 * self.max_linear_speed
                self.linear_speed = max(self.linear_speed, -self.max_linear_speed)
                self.ispressed = True
                ischanged = True
            if key == 'a':
                self.angular_speed = self.angular_speed + 0.1 * self.max_angular_speed
                self.angular_speed = min(self.angular_speed, self.max_angular_speed)
                self.ispressed = True
                ischanged = True
            if key == 'd':
                self.angular_speed = self.angular_speed - 0.1 * self.max_angular_speed
                self.angular_speed = max(self.angular_speed, -self.max_angular_speed)
                self.ispressed = True
                ischanged = True
            if key == 's':
                self.linear_speed = 0.0
                self.angular_speed = 0.0
                self.ispressed = True
                ischanged = True
            if key == 'p':
                self.pause_motion = True
                self.mode = 'pause'
                self.ispressed = False
                ischanged = True
            if key == 'm':
                self.pause_motion = False
                self.mode = 'move '
                self.ispressed = False
                ischanged = True
            if key == 'c':
                self.continuous_control = True
                ischanged = True
            if key == 'n':
                self.continuous_control = False
                ischanged = True
            if (key == 'q') or (key == '\x03'):
                self.exit = True

            if ischanged:
                kInput = kInput + 1
                if kInput > 10:
                    kInput = 0
                    self.print_instruction()
                self.print_settings()

    def publish_commands(self):
        twist_cmd = Twist()
        pause_cmd = Bool()

        if not self.pause_motion:
            pause_cmd.data = self.pause_motion
            self.pause_pub.publish(pause_cmd)

        if not self.pause_motion and (self.continuous_control or self.ispressed):
            self.ispressed = False
            twist_cmd.linear.x = self.linear_speed
            twist_cmd.angular.z = self.angular_speed
            self.twist_pub.publish(twist_cmd)

    def reset_controls(self):
        # Resets the teleop controls.
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.pause_motion = True
        self.mode = 'pause'
        self.ispressed = False
        self.print_settings()


def main(args=None):
    rclpy.init(args=args)
    neslisah_teleop_key = NeslisahTeleopKey()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(neslisah_teleop_key)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        neslisah_teleop_key.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
