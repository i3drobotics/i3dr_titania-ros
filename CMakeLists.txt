
cmake_minimum_required(VERSION 2.8.3)
project(i3dr_titania)

find_package(catkin REQUIRED roscpp std_msgs tf rospy message_generation)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS
)

link_directories()
add_definitions(-std=c++11)

set (vimba_install_cmd "./vimba/install.sh")
message(STATUS "Running Vimba install command: ${vimba_install_cmd}")
execute_process(COMMAND ${vimba_install_cmd}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  OUTPUT_VARIABLE vimba_install_output)

message(STATUS "Vimba install response: ${vimba_install_output}")