
cmake_minimum_required(VERSION 2.8.3)
project(i3dr_titania)

find_package(catkin REQUIRED roscpp std_msgs tf rospy message_generation)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Download and install Vimba SDK in package directory using 'install.sh' script
# Will also set GENICAM_GENTL64_PATH environment variable for vimba USB and GigE TL
# This will be skipped if the Vimba_4_2 folder already exists
set (vimba_install_cmd "./vimba/install.sh")
message(STATUS "Running Vimba install command: ${vimba_install_cmd}")
execute_process(COMMAND ${vimba_install_cmd}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  OUTPUT_VARIABLE vimba_install_output)
message(STATUS "Vimba install response: ${vimba_install_output}")

# Find Vimba package (using FindVimba.cmake)
find_package(Vimba)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${vimba_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS
)

link_directories()
add_definitions(-std=c++11)

add_executable(vimba_api_node src/vimba_api_node.cpp)
target_link_libraries(vimba_api_node 
  ${catkin_LIBRARIES}
  ${vimba_LIBRARIES}
)