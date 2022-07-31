#!/bin/bash

set -e

echo "Starting install preparation"

sudo apt-get -y install git

echo ""
echo "cloning the repository"
echo ""

cd
mkdir git
cd git
git clone https://github.com/ctu-mrs/summer-school-2022
cd summer-school-2022
echo "running the main install.sh"
./install.sh

echo ""
echo "installation part ended"
echo ""
