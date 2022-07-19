#!/usr/bin/env python
from math import pi
TWOPI = 2*pi 
START_DELAY = 2

# ------------ MASTER ---------------    
STATE_TAKEOFF = 0 # takeoff from hector's base
STATE_TURTLE = 1 # fly to turtle
STATE_GOAL = 2 # fly to turtle's end goal
STATE_BASE = 3 # fly to hector's base
STATE_LAND = 4 # land at hector's base
STATE_DONE = 5 # done with task
CRUISE_ALTITUDE = 2.0
CLOSE_ENOUGH = 0.3
CLOSE_ENOUGH_SQ = CLOSE_ENOUGH**2
TOO_FAR = 0.5
TOO_FAR_SQ = TOO_FAR**2
TARGET_SEPARATION = 0.5

# ------------ MOTION ---------------
USE_GROUND_TRUTH = False
RAD_EQUATOR = 6378137. # m
RAD_POLAR = 6356752.3 # m
G = 9.80665 # Acceleration due to gravity. m/s/s
DEG2RAD = pi/180. # GPS is in terms of degrees
# Noises are derived from topics. Please refer to them to get a better estimate
IMU_NX = 40 # IMU linear acceleration x noise. (m/s/s)^2
IMU_NY = 40 # IMU linear acceleration y noise. (m/s/s)^2
IMU_NZ = 1 # IMU linear acceleration z noise. (m/s/s)^2
IMU_NO = 0.5 # IMU angular velocity z noise. (rad/s)^2
# GPS topic data in lat (deg) long (deg) alt (m).
GPS_NX = 0.02 # GPS noise in x. Derive this by examining noise in calculated x. (m)^2
GPS_NY = 0.02 # GPS noise in y. Derive this by examining noise in calculated y. (m)^2
GPS_NZ = 0.04 # GPS noise in z. Derive this by examining noise in calculated z. (m)^2
ALT_NZ = 0.5 # Altimeter noise, can be changed. (m)^2
MAG_NO = 0.000683 # Magnetic noise, can be changed. (rad/s)^2

# ------------ MOVE ---------------    
DISABLE_MOVE = False
KP_X = KP_Y = KP_V = 4.0
KI_X = KI_Y = KI_V = 0.1
KD_X = KD_Y = KD_V = 0.3
KP_Z = 6.0
KI_Z = 0.1
KD_Z = 3.0
MAX_V = 1
MAX_W = 0.2*pi
MAX_ASCEND = 1.
MAX_DESCEND = -0.3
