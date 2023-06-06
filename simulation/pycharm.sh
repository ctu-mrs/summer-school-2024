#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./singularity.sh exec "source ~/.bashrc && /opt/pycharm-community-2022.1.3/bin/pycharm.sh ~/summer-school-2023/mrim_task/mrim_planner/ > /dev/null 2>&1 &"
