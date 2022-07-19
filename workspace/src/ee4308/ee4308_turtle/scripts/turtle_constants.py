#!/usr/bin/env python
from math import pi, sqrt

# General
DEG2RAD = [i/180.0*pi for i in xrange(360)]
SQRT2 = sqrt(2.)
TWOPI = 2.*pi
HALFPI = 0.5*pi

# Turtle Mapper
L_OCC = 1 # log_odds for occupied cell, but scaled to an integer.
L_FREE = -L_OCC # assume log_odds for free cell is negative of log_odds for occupied cell
L_THRESH = 10 # <= L_THRESH means free; >= L_THRESH means occupied; in between is unknown. Scaled proportionally to L_OCC and L_FREE
L_MAX = 30 # limit possible values of log odds in occupancy grid to between -L_MAX and L_MAX. Scaled proportionally to L_OCC and L_FREE
COST_MAP_FREE = 101 # used for coloring and identifying free cells on a map
COST_MAP_OCC = 0 # used for coloring and identifying occupied cells on a map
COST_MAP_UNK = -1 # used for coloring and identifying unknown cells on a map
COST_MAP_INF = 60 # used for coloring and identifying inflated cells on a map

# Turtle Master
CLOSE_ENOUGH = 0.1 # threshold to consider close enough to target / turningpoint / goal (m)
CLOSE_ENOUGH_SQ = CLOSE_ENOUGH*CLOSE_ENOUGH # (m^2)
PATH_PLANNER = "THETA*" # "A*" or "THETA*"
COST_FUNCTION = "EUCLIDEAN" # "DIAGONAL" or "EUCLIDEAN"

# Turtle Motion
AXLE_TRACK = 0.16 #m
WHEEL_RADIUS = 0.033 #m

# Turtle Move
# constraints
# MAX_V and MAX_W are set at 0.22 and 2.84 respectively. cannot be changed.
MAX_DV = 0.05 # maximum change in forward speed (m/s) in one ITERATION_PERIOD
MAX_DW = 0.2 # maximum change in angular speed (rad/s) in one ITERATION_PERIOD
# define PD controller gains
KP_V = 4.0 # P gain for forward velocity
KP_W = 4.0 # P gain for angular velocity
KD_V = 16.0 # D gain for forward velocity
KD_W = 10.0 # D gain for angular velocity
DISABLE_MOVE = False
