#!/bin/bash
# change to current ee4308 workspace
export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

# set the world and its parameters
source world.sh

# assign permissions
source devel/setup.bash
chmod +x src/ee4308/ee4308_bringup/scripts/*.py
chmod +x src/ee4308/ee4308_bringup/scripts/*.sh
chmod +x src/ee4308/ee4308_turtle/scripts/*.py
chmod +x src/ee4308/ee4308_hector/scripts/*.py
chmod +x *.sh

# bring up gazebo and build all models
roslaunch ee4308_bringup project2_init.launch
