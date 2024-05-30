from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_merger',  # Replace with the name of your package
            executable='laser_scan_merger',  # Replace with the name of your node executable
            name='laser_scan_merger',
            output='screen',
            parameters=[
                # Add any ROS parameters here if needed
            ]
        )
    ])
