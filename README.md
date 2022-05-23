# jackal_kinova_simulator Melodic version

## Overview
This is a mobile manipulator simulator package using Gazebo, RViz, MoveIt, move_base.

The model of the mobile manipulator robot was created by combining [Kinova Robotics' Gen3 Lite](https://github.com/Kinovarobotics/ros_kortex) and [Clearpath Robotics' Jackal](https://github.com/jackal/jackal.git).

This package is for ROS Melodic.


## Installation

### Dependencies
This software is built on the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Installation)).

- For Mobile Manipulator Dependencies:
```
- jackal mobile robot dependencies
$ sudo apt-get install ros-melodic-jackal-*
$ sudo apt-get install ros-melodic-ddynamic-reconfigure

- ros_controllers
$ sudo apt-get install ros-melodic-ros-controllers
```

- For [MoveIt](https://moveit.ros.org/) install:
```
$ sudo apt-get install ros-melodic-moveit
```

- This package uses same ```ar_track_alvar``` package with our [husky_ur3_simulator](https://github.com/QualiaT/husky_ur3_simulator.git) package. For [ar_track_alvar package](https://github.com/QualiaT/ar_track_alvar) install:
```
$ cd ~/catkin_ws/src
$ git clone -b melodic-devel https://github.com/QualiaT/ar_track_alvar.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```


- ```kortex_driver``` in [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) is required to simulate Gen3 Lite, and [conan](https://docs.conan.io/en/latest/installation.html) is required to build [ros_kortex](https://github.com/Kinovarobotics/ros_kortex). 
```
- conan
$ python -m pip install conan
$ source ~/.profile
$ echo "export CONAN_REVISIONS_ENABLED=1" >> ~/.bashrc
$ source ~/.bashrc

- ros_kortex
$ cd ~/catkin_ws/src
$ git clone -b melodic-devel https://github.com/Kinovarobotics/ros_kortex.git
```

- If [gazebo_grasp_fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs) is already installed, remove ```gazebo-pkgs``` in ```ros_kortex/third_party``` before running ```catkin_make```.


### jackal_kinova_simulator Installation
```
$ cd ~/catkin_ws/src
$ git clone -b melodic-devel https://github.com/Sungwwoo/jackal_kinova_simulator.git

$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```




## How to Use
```
- Bring up Gazebo with the robot model
$ roslaunch jackal_kinova_gazebo HRI_lab.launch

- Bring up MoveIt & RViz
$ roslaunch jackal_kinova_moveit_config Omni_control.launch

- If you want to navigation using map, type the following command.
$ roslaunch jackal_kinova_navigation HRI_lab.launch
```
