cmake_minimum_required(VERSION 3.0.2)

project(mbs_waypoint_follower)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)
set (GeographicLib_DIR "/usr/local/lib/cmake/GeographicLib")

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  rospy
  roscpp
  smach
  std_msgs
  tf
  roslaunch
  GeographicLib REQUIRED
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs rospy smach std_msgs roscpp
)

catkin_python_setup()

# Add the given directories to those the compiler uses to search for include files. 
# So nothing but giving the location of the include files
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


# Adding the executable files for the build
add_executable(gps_move_base_node src/gps_move_base_node.cpp)

# linking the libraries for successful binary genertion
target_link_libraries(gps_move_base_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


catkin_install_python(
  PROGRAMS nodes/follow_waypoints
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/collect_waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

