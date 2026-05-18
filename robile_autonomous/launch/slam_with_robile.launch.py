from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Find the robot_localization package
    robot_localization_share = FindPackageShare('robot_localization')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # --- STEP 1: Robot Simulation and SLAM ---
        # This section launches the robot in Gazebo and starts SLAM Toolbox.
        # Keep this uncommented for Test 1.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('robile_gazebo'),
                    'launch',
                    'gazebo_4_wheel.launch.py'
                ])
            ),
            # Pass use_sim_time to the included launch file if it supports it
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('robile_autonomous'),
                    'config',
                    'slam_params.yaml'
                ]),
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # --- NEW: Robot Localization EKF Node ---
        # This node fuses odometry and SLAM data to provide a robust pose estimate.
        # It will publish a PoseStamped message to /current_pose.
        # Uncomment this section AFTER verifying Step 1.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('robile_autonomous'),
                    'config',
                    'ekf.yaml' # Path to your EKF configuration file
                ]),
                {'use_sim_time': use_sim_time}
            ],
            # Remap the EKF's filtered odometry output to /current_pose
            # The ekf_node publishes nav_msgs/msg/Odometry to /odometry/filtered.
            # We want a PoseStamped message, so we will remap /odometry/filtered
            # to /current_pose. The navigation_controller expects PoseStamped,
            # but it will likely only use the pose part from the Odometry message.
            # If your navigation_controller strictly needs PoseStamped and not Odometry,
            # you might need a small bridge node, but often it can handle Odometry.
            # Let's assume it can handle the PoseWithCovarianceStamped part of Odometry.
            # If not, we'd need a separate node to extract PoseStamped.
            # For simplicity, we'll remap the output topic.
            remappings=[
                ('/odometry/filtered', '/current_pose')
            ]
        ),

        # --- STEP 2: A* Global Planner ---
        # Uncomment this section AFTER verifying Step 1 and the EKF node (new).
        Node(
            package='robile_autonomous', # Replace with your actual package name if different
            executable='A_star_planner',
            name='astar_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # --- STEP 3: Navigation Controller ---
        # Uncomment this section AFTER verifying Step 2.
        Node(
            package='robile_autonomous', # Replace with your actual package name if different
            executable='navigation_controller',
            name='navigation_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # --- STEP 4: Potential Field Local Planner ---
        # Uncomment this section AFTER verifying Step 3.
        Node(
            package='robile_autonomous', # Replace with your actual package name if different
            executable='potential_field_planner',
            name='potential_field_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
