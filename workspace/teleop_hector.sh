#!/bin/bash 
# must run proj2_init.sh in another terminal first

# source the workspace
source devel/setup.bash

#rosservice call --wait /uav/enable_motors true # enable motors for uav
rosrun ee4308_bringup teleop_hector.py
