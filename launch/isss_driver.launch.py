from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isss_ros2_driver',
            executable='isss_ros2_driver_node',
            name='isss_ros2_driver_node',
            output='screen',
            parameters=[{
                'save_path': '/root/ros2_ws/src/isss_ros2_driver/isss_data/',
                'range': 20.0
            }]
        )
    ])
