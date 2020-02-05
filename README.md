# Multi Robot UA (Tutorial #0)
Launching multiple turtlbots at the same time.
This [tutorial](https://www.theconstructsim.com/ros-qa-130-how-to-launch-multiple-robots-in-gazebo-simulator/) was followed during the development of this project.

## Requirements
This repository was tested using:
- Ubuntu 18.04 LTS
- [Ros Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Installation
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/nestoregon/multi_robot_ua.git
```

3 packages were used to run the turtlebot3
- [turtlebot](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

Run the following commands into your ```src``` folder to download the files and include the turtlebot3 into your catkin workspace

```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations
cd ..
catkin_make
```
Source the environment to be able to locate the packages.
```bash
source devel/setup.bash # for bash
source devel/setup.zsh  # for zsh
```

## Run the code
Once we have undergone a successful installation the following code should run:
```bash
roslaunch multi_robot main.launch   #for 2 robots
roslaunch multi_robot main_4.launch #for 4 robots
```

Enjoy!
