cmake_minimum_required(VERSION 3.1.2)
project(mrim_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mrim_resources
  )

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime mrim_resources
  LIBRARIES mrim_planner
  )
