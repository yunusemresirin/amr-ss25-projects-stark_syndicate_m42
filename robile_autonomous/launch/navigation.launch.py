from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SLAM
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{'use_sim_time': True}]
        ),
        
        # A* Global Planner
        Node(
            package='robile_autonomous',
            executable='astar_planner',
            name='astar_planner'
        ),
        
        # Potential Field Local Planner
        Node(
            package='robile_autonomous',
            executable='potential_field_planner',
            name='potential_field_planner'
        ),
        
        # Navigation Controller
        Node(
            package='robile_autonomous',
            executable='navigation_controller',
            name='navigation_controller'
        ),
    ])