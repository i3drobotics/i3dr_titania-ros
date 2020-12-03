# i3dr_titania-ros
ROS driver for Titania stereo camera system from Industrial 3D Robotics 

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
