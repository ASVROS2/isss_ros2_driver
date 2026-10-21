from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('isss_ros2_driver')
    default_config = os.path.join(pkg_share, 'config', 'isss_driver.yaml')

    config_file = LaunchConfiguration('config_file').perform(context) or default_config
    params = [config_file]

    overrides = {}
    ip = LaunchConfiguration('ip').perform(context)
    range_val = LaunchConfiguration('range').perform(context)
    save_path = LaunchConfiguration('save_path').perform(context)

    if ip:
        overrides['ip'] = ip
    if range_val:
        overrides['range'] = float(range_val)
    if save_path:
        overrides['save_path'] = save_path

    if overrides:
        params.append(overrides)

    return [
        Node(
            package='isss_ros2_driver',
            executable='isss_ros2_driver_node',
            name='isss_ros2_driver_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=params,
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('isss_ros2_driver')
    default_config = os.path.join(pkg_share, 'config', 'isss_driver.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace for the driver node and its topics/services'),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Full path to the driver parameter YAML file'),
        DeclareLaunchArgument(
            'ip',
            default_value='',
            description='Optional sonar IP override (empty = use YAML value)'),
        DeclareLaunchArgument(
            'range',
            default_value='',
            description='Optional range override in meters (empty = use YAML value)'),
        DeclareLaunchArgument(
            'save_path',
            default_value='',
            description='Optional directory for binary logs (empty = use YAML value)'),
        OpaqueFunction(function=launch_setup),
    ])
