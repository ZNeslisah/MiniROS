from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    

    pipe_read = Node(
            package='mini_control',
            executable='pipe_read',
            name='pipe_read_node',
            output='screen'
        )    
    pipe_write = Node(
            package='mini_control',
            executable='pipe_write',
            name='pipe_write_node',
            output='screen'
        )
    battery = Node(
            package='mini_control',
            executable='battery',
            name='battery_node',
            output='screen'
        )
    neopixel =Node(
            package='mini_control',
            executable='neopixel',
            name='neopixel_node',
            output='screen'
        )
    
    
    return LaunchDescription([
        pipe_read,
        pipe_write,
        battery,
        neopixel
    ])
