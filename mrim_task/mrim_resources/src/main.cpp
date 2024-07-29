#include <ros/package.h>
#include <ros/ros.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "mrim_resources");

  ROS_INFO("It works!");

  return 0;
}
