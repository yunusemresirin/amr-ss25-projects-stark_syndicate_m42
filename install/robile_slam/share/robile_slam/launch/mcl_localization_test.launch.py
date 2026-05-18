from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    # Declare launch arguments first
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(
            os.getenv('ROS2_WS', '/ros2_ws'),
            'src/robile_slam/maps/dynamic_world_map.yaml'),
        description='Full path to map yaml file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Use LaunchConfiguration AFTER declaration
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # gazebo launch file
    gazebo_launch = Node(
        package='robile_gazebo',
        executable='gazebo_4_wheel.launch.py',
        name='gazebo',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file},
                    {'use_sim_time': use_sim_time}]
    )

    # AMCL node
    amcl_node = Node(
        package='robile_slam',
        executable='mcl',
        name='mcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            os.getenv('ROS2_WS', '/ros2_ws'),
            'src/robile_slam/config/rviz_amcl.rviz')]
    )

    return LaunchDescription([
        
        map_file_arg,
        use_sim_time_arg,
        map_server_node,
        # gazebo_launch,
        amcl_node,
        # rviz_node
    ])
