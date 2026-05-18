from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robile_slam",
            executable="global_planner_node",
            name="global_planner",
            output="screen"
        ),
        Node(
            package="robile_slam",
            executable="waypoint_follower_node",
            name="waypoint_follower",
            output="screen"
        ),
        Node(
            package="robile_slam",
            executable="potential_field_node",
            name="potential_field",
            output="screen"
        )
    ])
