cmake_minimum_required(VERSION 3.0.2)
project(gazebo-plugin-amcl-pose)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gazebo-plugin-amcl-pose
#  CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp rospy std_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
