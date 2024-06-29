from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():


    
    # # Nodes for camera and ARUCO detection
    # get_image = Node(
    #     package='image_tools',
    #     executable='cam2image',
    #     output='screen',
    #     name='get_image',
    #     # parameters=[{'camera_name': 'camera', 'image_path': /camera/image_raw}]
    # )

    get_image = Node(
        package='aruco_tracker',
        executable='get_image',
        name='get_image'
    )

    detect_aruco = Node(
        package='aruco_tracker',
        executable='detect_aruco',
        name='detect_aruco'
    )
    follow_aruco = Node(
        package='aruco_tracker',
        executable='follow_aruco',
        name='follow_aruco'
    )
    

    return LaunchDescription([
      get_image,
      detect_aruco,
      follow_aruco
    ])