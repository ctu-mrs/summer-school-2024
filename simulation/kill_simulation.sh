#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

./apptainer.sh exec "source ~/.bashrc && ( tmux -L mrs kill-session -t simulation; pkill roscore; pkill rviz;  echo done)"
