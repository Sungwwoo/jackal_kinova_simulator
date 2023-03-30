# jackal_kinova_simulator Noetic version

## Overview
This is a mobile manipulator simulator package using Gazebo, RViz, MoveIt, move_base.

The model of the mobile manipulator robot was created by combining [Kinova Robotics' Gen3 Lite](https://github.com/Kinovarobotics/ros_kortex) and [Clearpath Robotics' Jackal](https://github.com/jackal/jackal.git).

This package is for ROS Noetic.


## Installation

### Dependencies
This software is built on the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Installation)).

- For Mobile Manipulator Dependencies:
```
- jackal mobile robot dependencies
$ sudo apt-get install ros-noetic-jackal-*
$ sudo apt-get install ros-noetic-ddynamic-reconfigure

- ros_controllers
$ sudo apt-get install ros-noetic-ros-controllers
```

- For [MoveIt](https://moveit.ros.org/) install:
```
$ sudo apt-get install ros-noetic-moveit
```

- This package uses same ```ar_track_alvar``` package with our [husky_ur3_simulator](https://github.com/QualiaT/husky_ur3_simulator.git) package. For [ar_track_alvar package](https://github.com/QualiaT/ar_track_alvar) install:
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/QualiaT/ar_track_alvar.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```
```jackal_kinova``` uses two Realsense D435i.
- For Realsense SDK 2.0 install:
```
$ sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
$ sudo add-apt-repository "deb http://librealsense.intel.com/Debian/apt-repo focal main" -u

$ sudo apt-get install librealsense2-dkms
$ sudo apt-get install librealsense2-utils
$ sudo apt-get install librealsense2-dev
$ sudo apt-get install librealsense2-dbg

# test
$ realsense-viewer
```

- For Realsense ROS package install:
```
$ export ROS_VER=noetic
$ sudo apt-get install ros-noetic-realsense2-camera
$ cd ~/catkin_ws/src/
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
$ cd ~/catkin_ws/
$ catkin_make
```

- ```kortex_driver``` in [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) is required to simulate Gen3 Lite, and [conan](https://docs.conan.io/en/latest/installation.html) is required to build [ros_kortex](https://github.com/Kinovarobotics/ros_kortex). 
```
- conan
$ python3 -m pip install conan
# if catkin_make fails, try:
$ sudo python3 -m pip install conan

$ echo "export CONAN_REVISIONS_ENABLED=1" >> ~/.bashrc
$ source ~/.bashrc

- ros_kortex
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/Kinovarobotics/ros_kortex.git
```

- If [gazebo_grasp_fix plugin](https://github.com/JenniferBuehler/gazebo-pkgs) is already installed, remove ```gazebo-pkgs``` in ```ros_kortex/third_party``` before running ```catkin_make```.


### jackal_kinova_simulator Installation
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/Sungwwoo/jackal_kinova_simulator.git

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
## How to Set Manipulator Initial pose
- The manipulator initial pose is set by ```kortex_driver``` using ```move_group```. 
- To change initial pose, see [home_robot.py](https://github.com/Sungwwoo/jackal_kinova_simulator/blob/noetic-devel/jackal_kinova_moveit_config/scripts/home_robot.py).  In ```home_robot.py```, ```HOME_ACTION_IDENTIFIER``` determines the pose of the manipulator specified in [jackal_kinova.srdf](https://github.com/Sungwwoo/jackal_kinova_simulator/blob/noetic-devel/jackal_kinova_moveit_config/config/jackal_kinova.srdf). There are 3 identifiers: ```1.retract```, ```2.home```, ```3.vertical```. 
- To change initial pose, change the joint values in ```jackal_kinova.srdf``` and ```HOME_ACTION_IDENTIFIER``` in ```home_robot.py```.
- To add identifier for ```kortex_driver```, edit ```CreateDefaultActions``` in [kortex_arm_simulation.cpp](https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/src/non-generated/driver/kortex_arm_simulation.cpp#L584).
