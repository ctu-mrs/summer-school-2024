#!/bin/bash

# get the path to the current directory
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd $MY_PATH

cd images
wget -c https://nasmrs.felk.cvut.cz/index.php/s/gLr91iWRgfSgn7I/download -O mrs_uav_system.sif --no-check-certificate
