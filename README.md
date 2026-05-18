# Task 1 – A* + Potential Field Global Planner for Robile

This branch implements an A*-based global planner combined with a Potential Field planner for the Robile platform. It also integrates AMCL-based localization and waypoint following.


## 📥 Setup Instructions
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

Step 1 – Connect to Robile

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

### Step 4 – Map the Environment

Follow the official mapping tutorial: 
[Official Documentation](https://robile-amr.readthedocs.io/en/latest/source/Tutorial/Demo%20Mapping.html)

After mapping, save the map.

### Step 5 – Test Using AMCL Localization

### (i) Switch RViz Config

Load: robile_slam/config/mapping_config.rviz

### New Terminal : Run Particle Filter Node

`ros2 run robile_slam particle_filter_node`

### New Terminal : Run the Global Planner

`ros2 run robile_slam global_planner_node`

### New Terminal : Run the waypoint follower

`ros2 run robile_slam waypoint_follower_node`


### New Terminal: Launch the Map Server / AMCL Node

`ros2 launch robile_navigation localization.launch.py`

### Step 7 – Provide Initial Pose in RViz

- In RViz, select 2D Pose Estimate.

- Click and drag to align the estimated starting pose with the robot’s actual starting point.

- You may need to move the robot slightly until the pose in RViz coincides with the actual robot position.



## Step 8 – Set Goal Position

- Use 2D Goal Pose in RViz to set a target location on the map.

- The combined Global Planner + Potential Field Planner will generate a path.

- The robot will start moving toward the goal.