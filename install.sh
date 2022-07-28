#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

UBUNTU_OS="false"

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

# install singularity
if [ -f "/etc/os-release" ]; then

  OS_NAME=$(head -n 1 /etc/os-release)
  OS_NAME="${OS_NAME,,}"

  # Ubuntu: automated installation
  if [[ $OS_NAME == *"ubuntu"* ]]; then

    UBUNTU_OS="true"

    # install dependencies
    cd ${MY_PATH}/simulation
    ./01_install.sh

  fi

fi

# download singularity MRS system img
cd ${MY_PATH}/simulation
./02_download.sh

# compile workspace
cd ${MY_PATH}/simulation
./03_compile.sh

# print info
if [[ "${UBUNTU_OS}" == "false" ]]; then

    echo -e "${RED}The task requires Singularity, but we don't have automated installation of Singularity for non-Ubuntu OS. Install Singularity manually for your OS:${NC}"
    echo -e "${RED}  - Official how-to:                        https://docs.sylabs.io/guides/3.0/user-guide/installation.html${NC}"
    echo -e "${RED}  - Get inspired here if you have Linux OS: ${MY_PATH}/simulation/01_install.sh${NC}\n"
    echo -e "${RED}Once you have Singularity set up, you are set to start.${NC}"
    echo -e "${RED}  - You can verify the installation with: bash ${MY_PATH}/simulation/singularity.sh${NC}"
    echo -e "${RED}  - Task starting point:                  bash ${MY_PATH}/simulation/run_offline.sh${NC}"

else

  echo -e "${GREEN}All done.${NC}"
  echo -e "Task starting point: ${GREEN}bash ${MY_PATH}/simulation/run_offline.sh${NC}"

fi
