# i3dr_titania-ros
ROS driver for Titania stereo camera system from Industrial 3D Robotics.  
**WARNING: THIS PACKAGE IS IN ACTIVE DEVELOPMENT AND NOT CURRENTLY READY FOR USE**

## Status
![ROS Build](https://github.com/i3drobotics/i3dr_titania-ros/workflows/ROS%20Build/badge.svg?event=push)

## Installation

For an easy setup, a rosinstall file is provided in 'install' folder of this repo which can be used to get this package and it's dependent ros packages in your workspace. 
In your ROS workspace use the following command:
```
wstool init src https://raw.githubusercontent.com/i3drobotics/i3dr_titania-ros/master/install/i3dr_titania_https.rosinstall
```
If you already have a wstool workspace setup then use the following command instead:
```
wstool merge -t src https://raw.githubusercontent.com/i3drobotics/i3dr_titania-ros/master/install/i3dr_titania_https.rosinstall
wstool update -t src
```

If you do not use wstool, you can download the packages using the following command:
```
cd PATH_TO_ROS_WS/src
git clone https://github.com/i3drobotics/i3dr_titania-ros.git
git clone https://github.com/i3drobotics/i3dr_stereo_camera-ros.git
git clone https://github.com/i3drobotics/camera_control_msgs.git
git clone https://github.com/i3drobotics/pylon_camera.git
```

To install package dependences use rodep:
```
rosdep install --from-paths src --ignore-src -r -y
```

Build using catkin (tested with catkin_make and catkin_build):
```
catkin_make
or
catkin build
```

Plug in your Titania camera to your machine and use the following launch file to test:
```
roslaunch i3dr_titania titania.launch
```

To check everything is working add the paramter 'rviz':
```
roslaunch i3dr_titania titania.launch rviz:=true
```

# Vimba
Original plan was to use Vimba cameras however there is an issue with the Vimba API inside of ROS (detailed below).  
The code that used Vimba has been left in however only very basic listing of available cameras as a ros node is provided and we will be using Basler's Pylon API going forward.
## Issue
Adding ros code to Vimba executables causes no cameras to be found.  
This is demonstrated in this package with two executables 'vimba_list_cameras' and 'vimba_list_cameras_node'.  
The only difference between the two is the '_node' executable has the following line: 
```
ros::init(argc, argv, "vimba_api");
```
Otherwise there two are identical. However, when these are run with the following commands
```
rosrun i3dr_titania vimba_list_cameras
```
```
rosrun i3dr_titania vimba_list_cameras_node
```
The non '_node' executable reports correctly
```
Cameras found: 2
```
Whereas the '_node' executable reports 
```
Cameras found: 0
```
