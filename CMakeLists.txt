cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_my_controllers_pde4430_cw1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
