#!/bin/bash

RVIZ_GUI=1

while [ $# -gt 0 ] ; do
  case $1 in
    -n | --nogui) RVIZ_GUI=0 ;;
  esac
  shift
done

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./apptainer.sh exec "source ~/.bashrc && roslaunch mrim_planner planner.launch RUN_TYPE:=offline RVIZ_GUI:=$RVIZ_GUI SESSION_PROBLEM:=offline"
