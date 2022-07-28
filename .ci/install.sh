#!/bin/bash

set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install preparation"

sudo apt-get -y install git

echo "clone mrs_uav_system"
cd
mkdir git
cd git
git clone https://github.com/ctu-mrs/mrs_uav_system.git
cd mrs_uav_system
echo "running the main install.sh"
./install.sh -g "$HOME/git" -m "false"

echo "installing summer_school_2022 dependencies"
cd $GITHUB_WORKSPACE
./install.sh

echo "installation part ended"
