cmake_minimum_required(VERSION 3.1.2)
project(mrim_resources)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  mrs_msgs
  nodelet
  cmake_modules
  message_runtime
  message_generation
  gazebo_ros
  )

# include Gazebo
find_package(gazebo REQUIRED)

# custom message types
add_message_files(DIRECTORY msg FILES
  InspectionPoint.msg
  InspectionProblem.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  mrs_msgs
  )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs mrs_msgs message_runtime message_generation message_runtime
  LIBRARIES mrim_resources
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(mrim_resources src/main.cpp)

add_dependencies(mrim_resources
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(mrim_resources
  ${catkin_LIBRARIES}
  )

install(DIRECTORY gazebo_files/models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

