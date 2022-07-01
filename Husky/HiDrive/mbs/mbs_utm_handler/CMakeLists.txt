cmake_minimum_required(VERSION 3.0.3)
project(mbs_utm_handler)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)
set (GeographicLib_DIR "/usr/local/lib/cmake/GeographicLib")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  roslaunch  
  mbs_msgs
  GeographicLib REQUIRED
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(gps_handler src/gps_handler.cpp)

target_link_libraries(gps_handler ${catkin_LIBRARIES})