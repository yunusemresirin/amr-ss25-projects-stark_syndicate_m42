from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': './src/robile_slam/maps/lab_map.yaml'}, {'use_sim_time': False}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': False}, {'autostart': True}, {'node_names': ['map_server']}]
        ),
        # Particle Filter
        # Node(
        #     package='robile_slam',
        #     executable='particle_filter_node',
        #     name='particle_filter_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}, {'num_particles': 500}, {'laser_max_range': 10.0}]
        # ),
        # Master Navigation
        # Node(
        #     package='robile_slam',
        #     executable='nav_stack',
        #     name='nav_stack',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # ),
    ])