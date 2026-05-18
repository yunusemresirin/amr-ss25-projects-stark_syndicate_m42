from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    num_particles_arg = DeclareLaunchArgument('num_particles', default_value='200')
    run_test_arg = DeclareLaunchArgument('run_test_publisher', default_value='true')

    use_sim_time = LaunchConfiguration('use_sim_time')
    num_particles = LaunchConfiguration('num_particles')
    run_test = LaunchConfiguration('run_test_publisher')

    # include map_server (uses robile_slam/launch/map_server.launch.py)
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robile_slam'),
                                 'launch', 'map_server.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # MCL node (robile_slam package: monte_carlo_pf_node.py)
    mcl_node = Node(
        package='robile_slam',
        executable='particle_filter_node',
        name='mcl_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_particles': num_particles,
            # optional: motion/sensor params - override here ↓  
            'alpha1': 0.1, 'alpha2': 0.1, 'alpha3': 0.1, 'alpha4': 0.1,
            'z_hit': 0.8, 'z_short': 0.1, 'z_max': 0.05, 'z_rand': 0.05,
            'sigma_hit': 0.2, 'lambda_short': 0.1
        }]
    )

    # Test publisher script (calls the Python script directly)
    test_pub = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('robile_slam'),
                'scripts',
                'test_mcl_publisher.py'
            ])
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", run_test, "' == 'true'"]))
    )

    return LaunchDescription([
        use_sim_time_arg,
        num_particles_arg,
        run_test_arg,
        map_server,
        mcl_node,
        test_pub,
    ])