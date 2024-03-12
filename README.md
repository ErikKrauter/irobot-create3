# Playing around with multi-robot simulation

This is a ROS 2 simulation stack for the [iRobot® Create® 3](https://edu.irobot.com/create3) robot.
Both Ignition Gazebo and Classic Gazebo are supported.

Have a look at the [Create® 3 documentation](https://iroboteducation.github.io/create3_docs/) for more details on the ROS 2 interfaces exposed by the robot.

## Prerequisites

Required dependencies:

1. [ROS 2 galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
2. ROS 2 dev tools:
    - [colcon-common-extensions](https://pypi.org/project/colcon-common-extensions/)
    - [rosdep](https://pypi.org/project/rosdep/): Used to install dependencies when building from sources

The above mentioned dependencies can be installed by following these steps:
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


sudo apt update
sudo apt upgrade
sudo apt install ros-galactic-desktop
sudo apt install ros-dev-tools

sudo apt update
sudo apt install python3.8 python3.8-venv python3.8-dev

python3.8 -m venv ~/ros_py38_venv
source ~/ros_py38_venv/bin/activate

sudo apt-get install -y python3-pip
pip install -U colcon-common-extensions rosdep
pip install rosdep
pip install colcon-common-extensions
pip install lxml
pip install numpy

```
Besides the aforementioned dependencies you will also need Classic Gazebo

#### Classic Gazebo

Install [Gazebo 11](http://gazebosim.org/tutorials?tut=install_ubuntu)

Gazebo 11 and all additional packages can be installed by following these steps:
```bash
sudo apt install gazebo11
sudo apt install ros-galactic-gazebo-ros-pkgs
sudo apt-get install ros-galactic-control-msgs
sudo apt-get install ros-galactic-irobot-create-msgs
sudo apt-get install ros-galactic-joint-state-publisher
sudo apt-get install ros-galactic-xacro
sudo apt-get install ros-galactic-ros2-controllers
sudo apt-get install ros-galactic-gazebo-ros2-control
# for debugging
sudo apt-get install ros-galactic-rqt ros-galactic-rqt-graph

```

## Build

- Create a workspace if you don't already have one:

```bash
mkdir -p ~/create3_ws/src
```

- Clone this repository into the src directory from above.

- Navigate to the workspace and install ROS 2 dependencies with:

```bash
cd ~/create3_ws
sudo apt-get update
rosdep install --from-path src -yi
```

- Build the workspace with:

```bash
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

## Run

#### Classic Gazebo

##### Empty world

Create® 3 can be spawned in an empty world in Gazebo and monitored through RViz. Arbitrarily many robots can be spawned, each must be provided with a unique namespace argument. By specifying x, y, z, yaw you can determine the initial pose of the robot in the world.

```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py namespace:=robot1
ros2 launch irobot_create_gazebo_bringup create3_spawn.launch.py namespace:=robot2 x:=5.0 y:=3
```

You can command the robots to undock and activate the wall follow action with the following commands:

```bash
ros2 action send_goal /robot1/undock irobot_create_msgs/action/Undock "{}"
ros2 action send_goal /robot2/undock irobot_create_msgs/action/Undock "{}"

ros2 action send_goal /robot1/wall_follow irobot_create_msgs/action/WallFollow "{follow_side: 1, max_runtime: {sec: 70, nanosec: 0}}"
ros2 action send_goal /robot2/wall_follow irobot_create_msgs/action/WallFollow "{follow_side: 1, max_runtime: {sec: 70, nanosec: 0}}"
```

Additionally the repository contains a coverage behavior, which lets the robot explore the environement while avoiding collisions

```bash
ros2 action send_goal /coverage create3_examples_msgs/action/Coverage "{explore_duration:{sec: 500, nanosec: 0}, max_runtime:{sec: 1000,nanosec: 0}}"
```

Or in the multi-robot setting:

```bash
ros2 action send_goal /robot1/coverage create3_examples_msgs/action/Coverage "{explore_duration:{sec: 500, nanosec: 0}, max_runtime:{sec: 1000,nanosec: 0}}"
ros2 action send_goal /robot2/coverage create3_examples_msgs/action/Coverage "{explore_duration:{sec: 500, nanosec: 0}, max_runtime:{sec: 1000,nanosec: 0}}"
```

## Package layout

This repository contains packages for both the Classic and Ignition Gazebo simulators:

- `irobot_create_common` Packages common to both Classic and Ignition
    - `irobot_create_common_bringup` Launch files and configurations
    - `irobot_create_control` Launch control nodes
    - `irobot_create_coverage` non-systematic coverage algorithm with obstacle avoidance
    - `irobot_create_description`  URDF and mesh files describing the robot
    - `irobot_create_nodes` Nodes for simulating robot topics and motion control
    - `irobot_create_toolbox` Tools and helpers for creating nodes and plugins

- `irobot_create_gazebo` Packages used for the Classic Gazebo Simulator
    - `irobot_create_gazebo_bringup` Launch files and configurations
    - `irobot_create_gazebo_plugins` Sensor plugins



    - `irobot_create_gazebo_sim`  Metapackage


## Videos

#### Single Robot
The following video shows the case of a single robot exploring the environment with no collision, by utlizing the /coverage action.

https://github.com/ErikKrauter/irobot-create3/assets/53872755/cb4185eb-a1a4-46d2-a5e3-520c26aa2aa5


#### Two Robots
The following video shows two robots in the environment at the same time. One robot execute the /coverage action, while the other one executes the /wall_follow action. 
As can be seen, the obstacle avoidance does not work properly in the case of multiple robots. The reasons for it are unclear. I didn't have the time to debug this issue further.


https://github.com/ErikKrauter/irobot-create3/assets/53872755/a0bee877-f7ec-4c78-830c-703bf03b19ab


