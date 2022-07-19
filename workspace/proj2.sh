#!/bin/bash
# Must run proj2_init.launch in another first

# change to current ee4308 workspace
export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

# set the world and its parameters
source world.sh
export TURTLE_INF="0.25"
export TURTLE_CELL="0.1"

# source the workspace
source devel/setup.bash

# run the programs
roslaunch ee4308_bringup project2_main.launch
