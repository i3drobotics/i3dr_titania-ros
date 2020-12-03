#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vimba_api.hpp"

int main(int argc, char **argv, char **envp)
{
    // Adding this line causes no cameras to be found
    ros::init(argc, argv, "vimba_api");

    // List available vimba cameras
    listCameras();
    
    
    //ros::NodeHandle n;
    //if (ros::ok()){
    //}
    //ros::spin();
}