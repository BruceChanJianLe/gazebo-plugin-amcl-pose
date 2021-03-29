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

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gazebo-plugin-amcl-pose
#  CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp rospy std_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(amclPosePlugin SHARED src/plugin_amcl_pose.cpp)
target_link_libraries(amclPosePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})