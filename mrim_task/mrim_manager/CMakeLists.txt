cmake_minimum_required(VERSION 3.1.2)
project(mrim_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  nodelet
  cmake_modules
  mrim_resources
  )

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS nodelet rospy std_msgs geometry_msgs nav_msgs message_runtime mrim_resources
  LIBRARIES mrim_manager
  )

catkin_install_python(PROGRAMS ${python_scripts_DIR}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
