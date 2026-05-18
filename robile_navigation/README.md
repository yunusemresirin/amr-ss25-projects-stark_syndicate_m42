# robile_navigation
Navigation launch files for robile in gazebo and real robot


## Instalation

This package depends on the robile packages and different ros navigation packages for the launch files to run.

1. [Robile Description]()
2. [Robile Gazebo]()

Follow the [robile_gazebo]() instalation instructions and install the apt packages required for gazebo and launching robot in gazebo.
Here we install packages required for navigation.
~~~ sh
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-amcl ros-$ROS_DISTRO-map-server ros-$ROS_DISTRO-move-base ros-$ROS_DISTRO-dwa-local-planner

cd ~/catkin_ws/src
git clone https://github.com/mas-group/robile_description.git
git clone https://github.com/mas-group/robile_gazebo.git
git clone https://github.com/mas-group/robile_navigation.git
git clone https://github.com/mas-group/robile_slam.git

catkin build robile_description robile_gazebo
catkin build
source ~/catkin_ws/devel/setup.bash
~~~

## Usage

To start the gazebo simulator, use one of the launch files defined in the [launch/](launch/) directory as follows:

~~~ sh
roslaunch robile_gazebo 4_wheel_platform.launch use_kelo_tulip:=false
~~~

This will launch the robile in simulation. Now we will launch the navigation stack to localize in the map

~~~ sh
roslaunch robile_navigation navigation.launch
~~~

