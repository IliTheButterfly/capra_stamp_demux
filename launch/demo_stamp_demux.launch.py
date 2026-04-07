from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('capra_stamp_demux')
    config_file = os.path.join(package_share, 'config', 'demo_stamp_demux.yaml')

    return LaunchDescription([
        Node(
            package='capra_stamp_demux',
            executable='stamp_demux',
            name='stamp_demux_cmd_vel_unstamp',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='capra_stamp_demux',
            executable='stamp_demux',
            name='stamp_demux_cmd_vel',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='capra_stamp_demux',
            executable='stamp_demux',
            name='stamp_demux_flippers_unstamp',
            parameters=[config_file],
            output='screen'
        ),
        Node(
            package='capra_stamp_demux',
            executable='stamp_demux',
            name='stamp_demux_pointcloud2',
            parameters=[config_file],
            output='screen'
        ),
    ])
