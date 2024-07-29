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
ln -s $GITHUB_WORKSPACE summer-school-2024
cd summer-school-2024
echo "running the main install.sh"
./install.sh

echo ""
echo "installation part ended"
echo ""
