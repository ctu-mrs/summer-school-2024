#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./apptainer.sh exec "source ~/.bashrc && /opt/pycharm-community-2023.1.2/bin/pycharm.sh ~/summer-school-2024/mrim_task/mrim_planner/ > /dev/null 2>&1" &
