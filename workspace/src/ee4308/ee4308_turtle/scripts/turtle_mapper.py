#!/usr/bin/env python

import roslib, rospy, rospkg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from ee4308_bringup.msg import EE4308MsgMotion
from std_msgs.msg import Bool

from math import pi, sqrt, sin, cos, ceil
from line_of_sight import get_los_path

from tf.transformations import euler_from_quaternion
from turtle_constants import DEG2RAD, SQRT2, L_OCC, L_FREE, L_THRESH, L_MAX, \
    COST_MAP_FREE, COST_MAP_OCC, COST_MAP_UNK, COST_MAP_INF

import sys
import numpy

# ================================= PARAMETERS ========================================== 
ITERATION_PERIOD = 0.1
PUBLISH_EVERY_ITERATION = 10
MAX_SCAN_RANGE= 3.5 

# ================================= CONSTANTS ==========================================

# ================================= GLOBALS ==========================================  
# some globals not listed -- only the ones required in service functions are listed
x_min = 0.
y_min = 0.
num_j = 0
cell_size = 0.

# =============================== SUBSCRIBERS =========================================  
def subscribe_motion(msg):
    global msg_motion
    msg_motion = msg
    
def subscribe_scan(msg):
    # stores a 360 long tuple of LIDAR Range data into global variable rbt_scan. 
    # 0 deg facing forward. +1 deg is anticlockwise from 0.
    global rbt_scan
    rbt_scan = msg.ranges
    
def subscribe_stop(msg):
    global stop
    stop = msg.data
    
# ================================ SERVICE ===========================================
def gen_inf_mask(inf_radius, cell_size, num_j):
    # generates mask for inflation
    r = int(ceil(float(inf_radius) / cell_size)) # radius in terms of number of cells 
    R = inf_radius*inf_radius + 1e-6 # inf_radius squared + some tolerance to avoid floating pt. imprecision
    Z = [z*cell_size*z*cell_size for z in xrange(-r, r+1)] # vector of distance squared of nearby cells location, in metres

    k = -r*num_j - r # first relative k
    dk = num_j - r -r -1 # k change for every row
    mask = []
    for x2 in Z:
        for y2 in Z:
            if x2 + y2 <= R:
                mask.append(k)
            k += 1
        k += dk
    return mask
    
def x2i(x):
    return int(round(float(x - x_min) / cell_size))
def y2j(y):
    return int(round(float(y - y_min) / cell_size))
    
# ================================ BEGIN ===========================================
def mapper(map_boundaries, inf_radius, _cell_size):
    # ---------------------------------- INITS ----------------------------------------------
    
    # init node
    rospy.init_node('turtle_mapper')
    
    # Set the labels below to refer to the global namespace (i.e., global variables)
    # global is required for writing to global variables. For reading, it is not necessary
    global rbt_scan, msg_motion, stop
    
    # Initialise global vars
    msg_motion = None
    rbt_scan = None
    stop = None

    # Subscribers
    rospy.Subscriber('/turtle/scan', LaserScan, subscribe_scan, queue_size=1)
    rospy.Subscriber('/turtle/motion', EE4308MsgMotion, subscribe_motion, queue_size=1)
    rospy.Subscriber('/turtle/stop', Bool, subscribe_stop, queue_size=1)
    
    # Publishers
    pub_map = rospy.Publisher('/turtle/map', OccupancyGrid, latch=True, queue_size=1)
    
    # Wait for Subscribers to receive data.
    # ~ note imu will not publish if you press Ctrl+R on Gazebo. Use Ctrl+Shift+R instead
    while (stop is None or rbt_scan is None or msg_motion is None or rospy.get_time() == 0) and not rospy.is_shutdown():
        pass
        
    if rospy.is_shutdown():
        return
    
    # inits
    global x_min, y_min, cell_size, num_j, num_i, num_k
    cell_size = _cell_size
    msg_map = OccupancyGrid();
    x_min = map_boundaries[0]
    y_min = map_boundaries[1]
    num_i = int(ceil(float(map_boundaries[2] - x_min) / cell_size)) + 1 # number of cells in i-axis of map (height)
    num_j = int(ceil(float(map_boundaries[3] - y_min) / cell_size)) + 1 # number of cells in j-axis of map (width)
    num_k = num_i*num_j # number of cells total
    cost_map = msg_map.data = numpy.ones(num_k, numpy.int8) * COST_MAP_UNK # published over topic /turtle/map. cost_map is now pointed to msg_map.data
    occ_grid = numpy.zeros(num_k) # internal log-odds binary occ grid
    inflation_layer = numpy.zeros(num_k, numpy.uint16) # inflation layer: 0 means no inflation. +ve means inflation
    
    # for costmap to display on rviz, need to fill in metadata at msg_map.info (google nav_msgs/OccupancyGrid)
    msg_map.info.resolution = cell_size
    msg_map.info.width = num_j
    msg_map.info.height = num_i
    msg_map.info.origin.position.x = -cell_size*0.5 + map_boundaries[0]
    msg_map.info.origin.position.y = -cell_size*0.5 + map_boundaries[1]
    msg_map.info.origin.position.z = 0.15
    # flip the map bcos it is not displayed properly in rviz by rotating via quaternion
    msg_map.info.origin.orientation.x = 1./SQRT2
    msg_map.info.origin.orientation.y = 1./SQRT2
    msg_map.info.origin.orientation.z = 0
    msg_map.info.origin.orientation.w = 0
    pub_map.publish(msg_map) # publish to ping it is working
        
    # create inflation mask
    inflation_mask = gen_inf_mask(inf_radius, cell_size, num_j)
    
    
    print('=== [TTL MAPPER] Initialised ===')
    t = rospy.get_time()
    publish_iter = 0
    while not rospy.is_shutdown() and not stop:
        if rospy.get_time() > t:
            
            # robot positions and index
            rx = msg_motion.x
            ry = msg_motion.y
            ro = msg_motion.o
            
            # convert to i, j, k
            ri = x2i(rx)
            rj = y2j(ry)
            rk = ri*num_j + rj
            
            d = 0 # degree counter
            saw_obstacle = True
            for _scan_range in rbt_scan: # for each scan in one degree
                if _scan_range > MAX_SCAN_RANGE: # limit scan_range from inf to MAX_SCAN_RANGE
                    scan_range = MAX_SCAN_RANGE
                    saw_obstacle = False # no obstacle at scan_range
                else:
                    scan_range = _scan_range
                    saw_obstacle = True # obstacle at scan_range
                    
                # get the edge of the scan (inverse sensor model)
                rad = DEG2RAD[d] + ro
                ox = cos(rad) * scan_range + rx
                oy = sin(rad) * scan_range + ry
                oi = x2i(ox)
                oj = y2j(oy)
                ok = oi*num_j + oj
                
                los_path = get_los_path(ri, rj, rk, oi, oj, ok, num_j)
                if saw_obstacle:
                    if los_path:
                        del los_path[-1] # remove last cell (oi, oj, ok), which is an obstacle, so we do not update it as free
                    else:
                        # no path, robot and obstacle on same cell
                        pass
                    
                for k in los_path: # for each cell from robot to edge of scan                   
                    # ignore if threshold exceeded
                    if occ_grid[k] <= -L_MAX:
                        continue
                    
                    # get previous label in cost map
                    tmp = cost_map[k]
                    
                    # update occ_grid as L_FREE
                    occ_grid[k] += L_FREE
                    
                    # update cost map
                    if tmp == COST_MAP_OCC and occ_grid[k] < L_THRESH:
                        # was occupied -> is unknown; relabel current cell in cost map
                        if inflation_layer[k] > 0:
                            cost_map[k] = COST_MAP_INF # mark as inflation zone if it is part of inflation
                        else:
                            cost_map[k] = COST_MAP_UNK # mark as unknown
                        
                        # was occupied -> is unknown; remove inflation layer caused by current on nearby cells
                        for relative_k in inflation_mask:
                            neighbor_k = relative_k + k # for all nearby cells, including current
                            
                            inflation_layer[neighbor_k] -= 1
                                
                            if inflation_layer[neighbor_k] == 0: # nearby cell inflation is removed
                                if occ_grid[neighbor_k] <= -L_THRESH: # is free
                                    cost_map[neighbor_k] = COST_MAP_FREE
                                else: # if not free, must be unknown; since must be >0 if it is occ
                                    cost_map[neighbor_k] = COST_MAP_UNK
                                
                            
                    elif tmp == COST_MAP_UNK and occ_grid[k] <= -L_THRESH:
                        # was unknown -> is free;
                        cost_map[k] = COST_MAP_FREE #relabel in cost map
                    
                # if there is an obstacle at the edge
                if saw_obstacle:
                    # update occ_grid as L_OCC
                    if occ_grid[ok] < L_MAX:
                        occ_grid[ok] += L_OCC
                    
                    # get previous label in cost map
                    tmp = cost_map[ok]
                    
                    if tmp != COST_MAP_OCC and occ_grid[ok] >= L_THRESH:
                        # was unknown / inflated / free -> is occupied; relabel
                        cost_map[ok] = COST_MAP_OCC
                        
                        # was unknown / inflated / free -> is occupied; add inflation layer caused by current on nearby cells
                        for relative_k in inflation_mask:
                            neighbor_k = relative_k + ok # for all nearby cells, including current
                            inflation_layer[neighbor_k] += 1
                            if cost_map[neighbor_k] != COST_MAP_OCC: # if nearby cell is not occ, then paint as inf
                                cost_map[neighbor_k] = COST_MAP_INF
                    
                    elif tmp == COST_MAP_FREE and occ_grid[ok] > -L_THRESH:
                        # was free -> is unknown (not already inflation / occupied)
                        cost_map[ok] = COST_MAP_UNK # relabel
                        
                # increment degree
                d += 1

    
            # track if within targetted iteration period
            et = (rospy.get_time() - t)
            if et > ITERATION_PERIOD:
                print('[TTL MAPPER] {}ms OVERSHOOT'.format(int(et*1000)))
            t += ITERATION_PERIOD
            
            # publish map every iteration
            publish_iter += 1
            if publish_iter >= PUBLISH_EVERY_ITERATION:
                publish_iter = 0 # reset counter
                pub_map.publish(msg_map) # publish to /turtle/map
                
        
if __name__ == '__main__':      
    try: 
        # parse goals
        if len(sys.argv) > 1:
            map_boundaries = sys.argv[1]
            map_boundaries = map_boundaries.split(',')
            for i in xrange(len(map_boundaries)):
                map_boundaries[i] = float(map_boundaries[i])
                
            mapper(map_boundaries, float(sys.argv[2]), float(sys.argv[3]))
        else:
            mapper([-10., -10., 10., 10.], 0.25, 0.1)
    except rospy.ROSInterruptException:
        pass

    print('=== [TTL MAPPER] Terminated ===')
