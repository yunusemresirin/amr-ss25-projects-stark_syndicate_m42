import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    localization_method_arg = DeclareLaunchArgument(
        'localization_method',
        default_value='slam',  # 'slam' or 'mcl'
        description='Localization method: slam (SLAM Toolbox) or mcl (Particle Filter MCL)'
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_method = LaunchConfiguration('localization_method')

    # Gazebo simulation (launch first)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robile_gazebo'),
                'launch',
                'gazebo_4_wheel.launch.py'
            ])
        )
    )

    amcl_pose_relay = Node(
        package='topic_tools',  # ROS2 version of relay is in 'topic_tools'
        executable='relay',
        name='amcl_pose_relay',
        output='screen',
        arguments=['/pose', '/amcl_pose'],  # republish /pose → /amcl_pose
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # SLAM Toolbox (only if localization_method == 'slam')
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(PythonExpression([
            '"', localization_method, '" == "slam"'
        ])),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robile_navigation'),
                'config',
                'mapper_params_online_async.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ]
    )

    # Monte Carlo Localization (only if localization_method == 'mcl')
    mcl_node = Node(
        package='robile_slam',
        executable='particle_filter_node',
        name='particle_filter_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            '"', localization_method, '" == "mcl"'
        ])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_particles': 500,
            'alpha1': 0.1,
            'alpha2': 0.1,
            'alpha3': 0.1,
            'alpha4': 0.1,
            'z_hit': 0.8,
            'z_short': 0.1,
            'z_max': 0.05,
            'z_rand': 0.05,
            'sigma_hit': 0.2,
            'lambda_short': 0.1
        }],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/cmd_vel', '/cmd_vel'),
            ('/initialpose', '/initialpose'),
            ('/amcl_pose', '/amcl_pose'),
            ('/particlecloud', '/particlecloud')
        ]
    )

    # A* Global Planner
    astar_planner = Node(
        package='robile_slam',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        respawn=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'inflation_radius': 0.3
        }],
        remappings=[
            ('/map', '/map'),
            ('/goal_pose', '/goal_pose'),
            ('/robot_pose', '/amcl_pose'),  # Uses AMCL or SLAM output
            ('/global_path', '/global_path')
        ]
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='robile_slam',
        executable='waypoint_follower_node',
        name='waypoint_follower_node',
        output='screen',
        respawn=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_tolerance': 0.5,
            'lookahead_distance': 1.0,
            'max_waypoint_distance': 2.0
        }],
        remappings=[
            ('/global_path', '/global_path'),
            ('/amcl_pose', '/amcl_pose'),
            ('/local_goal', '/local_goal'),
            ('/goal_reached', '/goal_reached')
        ]
    )

    # Potential Field Local Planner
    potential_field_planner = Node(
        package='robile_slam',
        executable='potential_field_node',
        name='potential_field_node',
        output='screen',
        respawn=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'k_attractive': 1.0,
            'k_repulsive': 2.0,
            'influence_radius': 1.5,
            'goal_tolerance': 0.2,
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
            'min_obstacle_distance': 0.3
        }],
        remappings=[
            ('/local_goal', '/local_goal'),
            ('/amcl_pose', '/amcl_pose'),
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        localization_method_arg,

        # Start Gazebo first
        gazebo_launch,

        # Start localization system
        amcl_pose_relay,
        slam_toolbox_node,
        mcl_node,

        # Navigation group
        GroupAction([
            astar_planner,
            waypoint_follower,
            potential_field_planner
        ])
    ])
