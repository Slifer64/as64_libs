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

PROJECT_NAME=$COLOR_CYAN$"as64_libs"$COLOR_RESET

MAIN_WS_DIR=${PWD}
INSTALL_SCRIPTS_DIR=$MAIN_WS_DIR/install
cd $INSTALL_SCRIPTS_DIR

if [ $# -eq 0 ]; then
  source main.sh
else
  source main.sh $1
fi
