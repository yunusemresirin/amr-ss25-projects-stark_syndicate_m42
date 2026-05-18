from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robile_gazebo'),
                'launch',
                'gazebo_4_wheel.launch.py'
            ])
        )
    )

    # SLAM Toolbox (only if localization_method == 'slam')
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robile_navigation'),
                'config',
                'mapper_params_online_async.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ]
    )

    localization_method_arg = DeclareLaunchArgument(
        'localization_method',
        default_value='slam',  # 'slam' or 'mcl'
        description='Localization method: slam (SLAM Toolbox) or mcl (Particle Filter MCL)'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        localization_method_arg,

        gazebo_launch,
        slam_toolbox_node,

        Node(
            package='robile_slam',
            executable='particle_filter_node',
            name='particle_filter_node',
            output='screen',
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
        ),
        
    ])
