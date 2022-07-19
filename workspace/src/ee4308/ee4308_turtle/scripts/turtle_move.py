#!/usr/bin/env python

import roslib, rospy, rospkg
from math import sqrt, cos, sin, pi, atan2, radians
import signal

from ee4308_bringup.msg import EE4308MsgMotion
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool

from turtle_constants import MAX_DV, \
    MAX_DW, KP_V, KP_W, KD_V, KD_W, TWOPI, HALFPI, DISABLE_MOVE #, TURN_THRESHOLD

# ================================= PARAMETERS ========================================== 
ITERATION_PERIOD = 0.05 # s

# ================================= CONSTANTS ==========================================        

# =============================== SUBSCRIBERS =========================================  
def subscribe_stop(msg):
    global stop
    stop = msg.data

def subscribe_target(msg):
    global msg_target
    msg_target = msg

def subscribe_motion(msg):
    global msg_motion
    msg_motion = msg

# ================================ SERVICES ===========================================
def limit_angle(angle):
    # limit angles to between -pi and pi, provided angle is between -2pi and 2pi
    if angle >= pi:
        return angle - TWOPI
    elif angle < -pi:
        return angle + TWOPI
    else:
        return angle

# ================================ BEGIN ===========================================
def move():
    # ---------------------------------- INITS ----------------------------------------------
    
    
    # Initialise last global var written in each subscribe handler so we know when all first messages are parsed
    global stop, msg_motion, msg_target, cmd_vel
    stop = None
    msg_motion = None
    msg_target = None
       
    # Subscribers
    rospy.Subscriber('/turtle/stop', Bool, subscribe_stop, queue_size=1)
    rospy.Subscriber('/turtle/target', PointStamped, subscribe_target, queue_size=1)
    rospy.Subscriber('/turtle/motion', EE4308MsgMotion, subscribe_motion, queue_size=1)
    
    # Publishers
    
    cmd_vel_lin = cmd_vel.linear # point to cmd_vel.linear so we can assign cmd_vel.linear.x quickly
    cmd_vel_ang = cmd_vel.angular # point to cmd_vel.angular so we can assign cmd_vel.angular.z quicklys
    
    # Wait for Subscribers to receive data.
    # bypass this loop if shutdown command is received
    while (stop is None or msg_motion is None or msg_target is None\
        or rospy.get_time() == 0) and not rospy.is_shutdown():
        pass
        
    # if shutdown received, quit
    if rospy.is_shutdown():
        return
    
    MAX_V = 0.22
    MAX_W = 2.84
    
    # initialise previous information   
    prev_cmd_v = 0.
    prev_cmd_w = 0.
    
    # get time
    t_iter = rospy.get_time()
    
    err_x = msg_target.point.x - msg_motion.x
    err_y = msg_target.point.y - msg_motion.y
    prev_err_pos = sqrt(err_x*err_x + err_y*err_y)
    prev_err_ang = limit_angle(atan2(err_y, err_x) - msg_motion.o)
    
    print('=== [TTL MOVE  ] Initialised ===')
    while not rospy.is_shutdown() and not stop:
        if rospy.get_time() > t_iter:            
            # ============================= GET POSE & TARGET ============================
            filter_x = msg_motion.x
            filter_y = msg_motion.y
            filter_o = msg_motion.o
            target_x = msg_target.point.x
            target_y = msg_target.point.y
            
            # ============================= GET ERRORS ============================
            err_x = target_x - filter_x
            err_y = target_y - filter_y
            err_pos = sqrt(err_x*err_x + err_y*err_y)
            err_ang = limit_angle(atan2(err_y, err_x) - filter_o)
            d_err_pos = err_pos - prev_err_pos
            d_err_ang = limit_angle(err_ang - prev_err_ang)
            
            # ============================= FIND FORWARD VELOCITY ============================
            # get forward velocity with triangle function
            cmd_v = cos(err_ang)**5 * (err_pos * KP_V + d_err_pos * KD_V)
                       
            # apply change in forward speed constraint
            if cmd_v - prev_cmd_v > MAX_DV:
                cmd_v = prev_cmd_v + MAX_DV
            elif cmd_v - prev_cmd_v < -MAX_DV:
                cmd_v = prev_cmd_v - MAX_DV
                
            # apply forward speed constraint
            if cmd_v > MAX_V:
                cmd_v = MAX_V
            elif cmd_v < -MAX_V:
                cmd_v = -MAX_V
                
            # ============================= FIND ANGULAR VELOCITY ============================
            # get angular velocity
            cmd_w = KP_W * sin(err_ang if err_ang < HALFPI and err_ang > -HALFPI else -err_ang)
            cmd_w += KD_W * d_err_ang
            
            # apply change in angular speed constraint
            if cmd_w - prev_cmd_w > MAX_DW:
                cmd_w = prev_cmd_w + MAX_DW
            elif cmd_w - prev_cmd_w < -MAX_DW:
                cmd_w = prev_cmd_w - MAX_DW
                
            # apply angular speed constraint
            if cmd_w > MAX_W:
                cmd_w = MAX_W
            elif cmd_w < -MAX_W:
                cmd_w = -MAX_W
                
            # ============================= MOVE ROBOT ============================
            # send command to move robot
            cmd_vel_lin.x = cmd_v
            cmd_vel_ang.z = cmd_w
            pub_cmd_vel.publish(cmd_vel)
                        
            # measure loop timing
            t_err = (rospy.get_time() - t_iter)
            if t_err > ITERATION_PERIOD:
                print('[TTL MOVE  ] {}ms OVERSHOOT'.format(int(t_err*1000)))
            t_iter += ITERATION_PERIOD
            
            # write prev information for next loop
            prev_cmd_v = cmd_v
            prev_cmd_w = cmd_w
            prev_err_ang = err_ang
            prev_err_pos = err_pos
    
    
def stop_motors_first(sig, frame):
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.
    cmd_vel.angular.z = 0.
    pub_cmd_vel.publish(cmd_vel)

    rospy.signal_shutdown("Stop")
    
if __name__ == '__main__' and not DISABLE_MOVE:      
    
    # init node
    rospy.init_node('turtle_move', disable_signals = True)
    
    signal.signal(signal.SIGINT, stop_motors_first)
    signal.signal(signal.SIGTERM, stop_motors_first)
    
    pub_cmd_vel = rospy.Publisher('/turtle/cmd_vel', Twist, latch=True, queue_size=1)
    cmd_vel = Twist()
        
    move()
    
    stop_motors_first(None, None)
        
    print('=== [TTL MOVE  ] Terminated ===')

