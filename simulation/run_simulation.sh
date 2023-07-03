#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

SESSION_NAME=simulation
if [ -z ${TMUX} ];
then
    TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
      echo "Starting new session."
    else
        echo "Already in tmux, leave it first."
          exit
fi


./singularity.sh exec "source ~/.bashrc && bash ~/summer-school-2023/simulation/tmux_scripts/simulation/start.sh"
