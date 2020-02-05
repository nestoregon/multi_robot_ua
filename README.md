# Multi Robot UA (Tutorial #0)
Launching multiple turtlbots at the same time.
This [tutorial](https://www.theconstructsim.com/ros-qa-130-how-to-launch-multiple-robots-in-gazebo-simulator/) was followed during the development of this project.

## Requirements
Tested using
- Ubuntu 18.04 LTS
- [Ros melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Install TurtleBot3 on your device
3 packages were used to run the turtlebot3
- [turtlebot](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

Run the following commands into your ```src``` folder to download them and include them into your catkin workspace

```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations
cd ..
catkin_make
```