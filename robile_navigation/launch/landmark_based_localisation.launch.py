import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    robile_nav_params = os.path.join(get_package_share_directory('robile_navigation'), 'config', 'robile_nav_params.yaml')

    static_transform_cmd = Node(package="tf2_ros",
                                executable="static_transform_publisher",
                                output="screen",
                                arguments=["0", "0", "0", "0", "0",
                                           "0", "map", "odom"]
                                )
    
    rfid_tag_finder_node = Node(
        package='robile_navigation',
        executable='robile_rfid_tag_finder.py',
        name='rfid_tag_finder',
        parameters=[robile_nav_params],
    )

    return LaunchDescription([
        rfid_tag_finder_node,
        static_transform_cmd
    ])