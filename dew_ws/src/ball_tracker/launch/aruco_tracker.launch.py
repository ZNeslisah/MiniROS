from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    get_image = Node(
        package='ball_tracker',
        executable='get_image',
        name='get_image'
    )
    detect_aruco = Node(
        package='ball_tracker',
        executable='detect_aruco',
        name='detect_aruco'
    )
    follow_aruco = Node(
        package='ball_tracker',
        executable='follow_aruco',
        name='follow_aruco'
    )
    

    return LaunchDescription([
      get_image,
      detect_aruco,
      follow_aruco
    ])