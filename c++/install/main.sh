#!/bin/bash

# ==================================
# define some colors for output
# ==================================
COLOR_RED="\033[1;31m"
COLOR_GREEN="\033[1;32m"
COLOR_YELLOW="\033[1;33m"
COLOR_BLUE="\033[1;34m"
COLOR_CYAN="\033[1;36m"
COLOR_WHITE="\033[1;37m"
COLOR_RESET="\033[0m"

cd $INSTALL_SCRIPTS_DIR
AS64_ERROR=0

declare -a arr=("eigen3" "armadillo" "frilibrary") # "eigen3" "armadillo" "frilibrary" "ati_sensor" "qt5")

echo -e $COLOR_BLUE"Installing dependencies for "$PROJECT_NAME $COLOR_RESET

#echo -e $COLOR_BLUE"Installing main Dependencies: cmake, wget, xz-utils..."$COLOR_RESET
#sudo apt-get update > /dev/null
#sudo apt-get install -y build-essential cmake wget xz-utils unzip > /dev/null

# mkdir -p deps

## now loop through the above array
for i in "${arr[@]}"
do
  cd $INSTALL_SCRIPTS_DIR
  source install_$i.sh
  if [ $AS64_ERROR -ne 0 ]; then
    echo -e $COLOR_RED"Failed to install dependencies for "$PROJECT_NAME $COLOR_RESET
    exit 1
  fi
  sleep 4
done

cd $INSTALL_SCRIPTS_DIR
#rm -rf deps/
cd ..

echo -e $PROJECT_NAME$COLOR_GREEN" dependencies installed successfully!"$COLOR_RESET
