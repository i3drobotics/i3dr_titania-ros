#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vimba_api.hpp"

int main(int argc, char **argv, char **envp)
{
    // List available vimba cameras
    listCameras();
}