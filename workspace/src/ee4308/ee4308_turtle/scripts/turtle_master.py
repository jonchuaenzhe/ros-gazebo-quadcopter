#!/usr/bin/env python

import roslib, rospy, rospkg
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool
from math import pi, sqrt, sin, cos, floor
from numpy import nan

from ee4308_bringup.msg import EE4308MsgMotion
from turtle_constants import CLOSE_ENOUGH_SQ, COST_MAP_FREE, COST_MAP_UNK, PATH_PLANNER, COST_FUNCTION
import path_planners
import sys

# ================================= PARAMETERS ========================================== 
if PATH_PLANNER == "A*":
    path_planner = path_planners.a_star
else:
    path_planner = path_planners.theta_star
ITERATION_PERIOD = 0.2
MAX_SCAN_RANGE = 3.5 # maximum scanning range of LIDAR

# ================================= CONSTANTS ==========================================        

# ================================= GLOBALS ==========================================        
# some globals not listed -- only the ones required in service functions are listed
num_i = 0
num_j = 0
x_min = 0.
y_min = 0.
cell_size = 0.
using_map = False

# =============================== SUBSCRIBERS =========================================  
def subscribe_motion(msg):
    global msg_motion
    msg_motion = msg

def subscribe_map(msg):
    if using_map:
        return
    global msg_map
    msg_map = msg
    
# =============================== SERVICE =========================================  
def planner2topic(path, msg):
    # path is list of integers of turning points from path planner: e.g. [32,123,412]
    # msg is the msg to publish, which is a Path() instance
    poses = []
    
    for idx in path:
        i = idx[0]
        j = idx[1]
        
        pose = PoseStamped() # create new pose
        position = pose.pose.position # point to position
        position.x = x_min + (i)*cell_size # convert i to x, #0.5 added for rviz
        position.y = y_min + (j)*cell_size # convert j to y, #0.5 added for rviz
        position.z = 0.18 # to see it float above the rviz, purely for visualisation
        poses.append(pose)

    msg.poses = poses
    
def x2i(x):
    return int(round(float(x - x_min) / cell_size))
def y2j(y):
    return int(round(float(y - y_min) / cell_size))
def i2x(i):
    return x_min + i*cell_size
def j2y(j):
    return y_min + j*cell_size

def plan(source_x, source_y, target_x, target_y):
    global using_map
    # convert x, y, coordinates to i, j to read occupancy grid in path planner
    source_i = x2i(source_x)
    source_j = y2j(source_y)
    target_i = x2i(target_x)
    target_j = y2j(target_y)
    
    using_map = True
    source = path_planners.a_star_mod(source_i, source_j, target_i, target_j, msg_map.data, 0, float(source_x - x_min) / cell_size, float(source_y - y_min) / cell_size)
    source_i = source[0]
    source_j = source[1]
    
    target = path_planners.a_star_mod(target_i, target_j, source_i, source_j, msg_map.data, 0.05)
    target_i = target[0]
    target_j = target[1]
    
    # plan using chosen planner
    path_planner(source_i, source_j, target_i, target_j, msg_map.data)
    using_map = False
    # path returned is reversed (goal at the start, robot position at the end of lists)
    

def is_free(cost_map_value):
    return cost_map_value == COST_MAP_FREE or cost_map_value == COST_MAP_UNK    
# ================================ BEGIN ===========================================
def master(goals=[]):
    # ---------------------------------- INITS ----------------------------------------------
    
                
    # init node
    rospy.init_node('turtle_master')
    
    # Set the labels below to refer to the global namespace (i.e., global variables)
    # global is required for writing to global variables. For reading, it is not necessary
    global msg_map, msg_motion
    
    # Initialise global vars
    msg_motion = None
    msg_map = None

    # Subscribers
    rospy.Subscriber('/turtle/motion', EE4308MsgMotion, subscribe_motion, queue_size=1)
    rospy.Subscriber('/turtle/map', OccupancyGrid, subscribe_map, queue_size=1)
    
    # Publishers
    # send reset to calibrate gyro and reset odom
    
    pub_path = rospy.Publisher('/turtle/path', Path, latch=True, queue_size=1) # for rviz
    pub_target = rospy.Publisher('/turtle/target', PointStamped, latch=True, queue_size=1)
    pub_stop = rospy.Publisher('/turtle/stop', Bool, latch=True, queue_size=1)
    msg_stop = Bool()
    msg_stop.data = False
    pub_stop.publish(msg_stop) # ping to others it is ready
    
    # Wait for Subscribers to receive data.
    # ~ note imu will not publish if you press Ctrl+R on Gazebo. Use Ctrl+Shift+R instead
    while (msg_motion is None or msg_map is None or rospy.get_time() == 0) and not rospy.is_shutdown():
        pass
        
    if rospy.is_shutdown():
        return
        
    # inits
    global num_i, num_j, cell_size, x_min, y_min, using_map
    tmp = msg_map.info
    num_j = tmp.width
    num_i = tmp.height
    cell_size = tmp.resolution
    tmp  = tmp.origin.position
    x_min = tmp.x + cell_size*0.5
    y_min = tmp.y + cell_size*0.5
    using_map = False
    
    path_planners.init_module(num_i, num_j)
    
    msg_path = Path();
    msg_path.header.frame_id = "map"
    
    msg_target = PointStamped()
    msg_target.header.frame_id = "map"
    msg_target.point.z = 0.18
    msg_target_position = msg_target.point
    
    # init goal position and index
    goal_idx = 0
    num_goals = len(goals)
    goal = goals[goal_idx]
    goal_x = goal[0]
    goal_y = goal[1]
    
    need_path = True
    
    
    turnpt_idx = 0
    
    print('=== [TTL MASTER] Initialised ===')
    
    t = rospy.get_time()
    run_start = t
    while not rospy.is_shutdown():
        if rospy.get_time() > t:
            # check path if there is overlap
            
            # replan the path
            plan(msg_motion.x, msg_motion.y, goal_x, goal_y)
            # print('[TTL MASTER] Path Found')
            if len(path_planners.path_pts) == 0:
                t += ITERATION_PERIOD
                continue
            
            # convert to appropriate data and publish for visualisation in rviz
            msg_path.header.seq += 1
            planner2topic(path_planners.path_pts, msg_path)
            pub_path.publish(msg_path)
                
            # get the first turning point (second point in path)
            turnpt_idx = len(path_planners.path_pts) - 1
            turnpt = msg_path.poses[turnpt_idx].pose.position
            turnpt_x = turnpt.x
            turnpt_y = turnpt.y
            msg_target_position.x = turnpt_x
            msg_target_position.y = turnpt_y
            pub_target.publish(msg_target)
            
            need_path = False
            
            # check if close neough to goal
            Dx = goal_x - msg_motion.x
            Dy = goal_y - msg_motion.y
            if Dx*Dx + Dy*Dy <= CLOSE_ENOUGH_SQ:
                goal_idx += 1
                if goal_idx == num_goals:
                    # print("[TTL MASTER] Final goal ({}, {}) reached!".format(goal_x, goal_y))
                    break
                    
                # print("[TTL MASTER] Goal ({}, {}) reached, new path requested".format(goal_x, goal_y))
                # get the next goal
                goal = goals[goal_idx]
                goal_x = goal[0]
                goal_y = goal[1]

                
                # generate new path
                need_path = True
            else:            
                # check if close enough to target
                Dx = turnpt_x - msg_motion.x
                Dy = turnpt_y - msg_motion.y
                if Dx*Dx + Dy*Dy <= CLOSE_ENOUGH_SQ:
                    # turnpt reached         
                    turnpt_idx -= 1
                    if turnpt_idx >= 0:                    
                        turnpt = path_planners.path_pts[turnpt_idx]
                        turnpt_x = i2x(turnpt[0])
                        turnpt_y = j2y(turnpt[1])   
                        msg_target_position.x = turnpt_x
                        msg_target_position.y = turnpt_y
                        pub_target.publish(msg_target)
                        # print("[TTL MASTER] Going to ({:6.2f}, {:6.2f})".format(turnpt_x, turnpt_y))  


                
            et = (rospy.get_time() - t)
            if et > ITERATION_PERIOD:
                print('[TTL MASTER] {}ms OVERSHOOT'.format(int(et*1000)))
            t += ITERATION_PERIOD
    
    msg_stop.data = True
    pub_stop.publish(msg_stop)
    rospy.sleep(1.) # sleep 1 second for topic to latch
    print('[TTL MASTER] {}s SECONDS ELAPSED'.format(rospy.get_time() - run_start))
    
        
if __name__ == '__main__':      
    try: 
        # parse goals
        if len(sys.argv) > 1:
            goals = sys.argv[1]
            goals = goals.split('|')
            for i in xrange(len(goals)):
                tmp = goals[i].split(',')
                tmp[0] = float(tmp[0])
                tmp[1] = float(tmp[1])
                goals[i] = tmp
            
            master(goals)
        else:
            master()
    except rospy.ROSInterruptException:
        pass
    print('=== [TTL MASTER] Terminated ===')

