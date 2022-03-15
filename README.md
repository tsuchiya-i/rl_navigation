# rl_navigation
 ROS navigation system based on deep reinforcement learning
 
## Python
Python version 2.7.
## Requirement
- tensorflow
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros)

## Installing
```
$ cd my_workspace
$ cd src
$ git clone https://github.com/tsuchiya-i/rl_navigation.git
$ cd ..
$ catkin_make
```

## How to use
Edit these files.
- launch/darknet_ros.launch
- launch/publiser_launch.launch
- launch/ddpg_navi.launch

```
$ roslaunch rl_navigation publiser_launch.launch
```
and
```
$ roslaunch rl_navigation ddpg_navi.launch
```
