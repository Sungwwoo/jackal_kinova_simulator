# jackal_kinova_simulator Noetic version

## Overview
This is a mobile manipulator simulator package using Gazebo, RViz, MoveIt, move_base.

The model of the mobile manipulator robot was created by combining Kinova Robotics' Gen3 Lite and Clearpath Robotics's Jackal.

This package is for ROS Noetic and not tested for Melodic.


## Installation

### Dependencies
This software is built on the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Installation)).

- For Jackal Mobile Robot Dependencies:
```
- jackal mobile robot dependencies
$ sudo apt-get install ros-noetic-jackal-*
$ sudo apt-get install ros-noetic-ddynamic-reconfigure

- ros_controllers
$ sudo apt-get install ros-noetic-ros-controllers
```

- To build kortex_driver package included in jackal_kinova_simulator, conan needs to be installed.
```
$ python3 -m pip install conan
$ echo "export CONAN_REVISIONS_ENABLED=1" >> ~/.bashrc
$ source ~/.bashrc
```

- For [MoveIt](https://moveit.ros.org/) install
```
$ sudo apt-get install ros-noetic-moveit
```

- This package uses same ar_track_alvar package with our [husky_ur3_simulator](https://github.com/QualiaT/husky_ur3_simulator.git) package.
- For [ar_track_alvar package](https://github.com/QualiaT/ar_track_alvar) install
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/QualiaT/ar_track_alvar.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```

### jackal_kinova_simulator install
```
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/Sungwwoo/jackal_kinova_simulator.git

$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```


### For Realsense SDK 2.0 Installation
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

### For Realsense ROS package Installation
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

## How to start?
```
- Bring up Gazebo with the robot model
$ roslaunch jackal_kinova_gazebo HRI_lab.launch

- Bring up MoveIt & RViz
$ roslaunch jackal_kinova_moveit_config Omni_control.launch

- If you want to navigation using map, type the following command.
$ roslaunch jackal_kinova_navigation HRI_lab.launch
```
