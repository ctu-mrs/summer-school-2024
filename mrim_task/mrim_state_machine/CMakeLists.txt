cmake_minimum_required(VERSION 3.1.2)
project(mrim_state_machine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nodelet
  cmake_modules
  mrs_lib
  mrs_msgs
  mavros_msgs
)

catkin_package(
   CATKIN_DEPENDS nodelet rospy std_msgs message_runtime mrs_msgs mrs_lib mavros_msgs
   LIBRARIES MrimStateMachine
   DEPENDS Eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(MrimStateMachine src/mrim_state_machine.cpp)

add_dependencies(MrimStateMachine
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrimStateMachine
  ${catkin_LIBRARIES}
  )
