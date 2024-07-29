#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./apptainer.sh exec "source ~/.bashrc && cd ~/summer-school-2024/simulation/user_ros_workspace && ([ ! -e .catkin_tools ] && catkin init || :) && catkin build"
