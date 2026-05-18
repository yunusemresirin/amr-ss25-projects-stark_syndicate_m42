# Task 3 – Automated Mapping & Environment Exploration

This branch implements automated mapping and environment exploration for the Robile platform. It allows the robot to autonomously map unknown environments while navigating through them.


## 📥 Step 1 - Setup Instructions
### 1. Clone into ROS 2 Workspace

Navigate to the src folder of your ROS 2 workspace and pull this repository directly (not inside a repo-named folder):

`cd ~/ros2_ws/src
git clone --depth 1 <repo_link> .`


### 2. Build the Workspace

`cd ~/ros2_ws
colcon build`

### 3. Source the Workspace

`source install/setup.bash`

## 🖥️ Running the System

### Step 1 – Connect to Robile

Connect your PC to the Robile5G Wi-Fi network.

Establish communication using the official documentation:

[Official Documentation](https://robile-amr.readthedocs.io/en/latest/source/Tutorial/Demo%20Communication.html)

### Step 2 – Launch Robile Drivers

`ros2 launch robile_bringup robot.launch.py`

### Step 3 – Visualize in RViz

Launch RViz2:

`rviz2`

Go to File → Open Config

Load: robile_slam/config/mapping_config.rviz

If the robot spawns near a wall or corner, use the teleop keyboard to move it toward the center of the arena.

### New Terminal : Run Particle Filter Node

`ros2 run robile_slam particle_filter_node`

### New Terminal : Run the Global Planner

`ros2 run robile_slam updated_global_planner`

### New Terminal : Run the waypoint follower

`ros2 run robile_slam waypoint_follower_node`


### New Terminal: Launch the MCL node

`ros2 run robile_slam mcl`

This should launch the monte carlo localization node that we have built and it would be waiting for the map data.

### New Terminal: Launch the exploration node

`ros2 run robile_slam exploration_node`

- This should launch the exploration node which would use the map data that is coming from the Lidar and explores the environment.

- The exploration node looks at the froniter and selects the farthest explorable point and moves towards it.



## New Terminal : Launch the Map server

`ros2 launch robile_navigation robile_nav2_bringup.launch.py`

The particles will get generated and MCL starts to look for initial pose but since it takes time to find the start pose on its own it would be better to give it as user input, when we provide the initial pose from Rviz it regenerates particles around that region and tries to find the actual start point.

### Step 7 – Provide Initial Pose in RViz

- In RViz, select 2D Pose Estimate.

- Click and drag to align the estimated starting pose with the robot’s actual starting point.

- You may need to move the robot slightly until the pose in RViz coincides with the actual robot position.



## Step 8 – Set Goal Position

- Use 2D Goal Pose in RViz to set a target location on the map.

- The combined Global Planner + Potential Field Planner will generate a path.

- The robot will start exploring the frontier while moving towards the goal, and map would generated on Rviz.

## Step 9 - Save the map using

`ros2 run nav2_map_server map_saver_cli -f map_name --occ 0.65 --free 0.15 --ros-args -p save_map_timeout:=20.0
`