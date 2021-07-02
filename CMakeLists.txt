
cmake_minimum_required(VERSION 2.8.3)
project(i3dr_titania)

find_package(catkin REQUIRED roscpp std_msgs tf rospy message_generation)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS
)

link_directories()
add_definitions(-std=c++11)