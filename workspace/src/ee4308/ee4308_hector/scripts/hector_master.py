#!/usr/bin/env python

import roslib, rospy, rospkg
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Int8
from math import sqrt, cos, sin, pi, atan2, asin
import sys

from hector_constants import *
from ee4308_bringup.msg import EE4308MsgMotion
# ================================= PARAMETERS ========================================== 
ITERATION_PERIOD = 0.1


# =============================== SUBSCRIBERS =========================================  
def subscribe_motion(msg):
    global msg_motion
    msg_motion = msg
       
def subscribe_turtle_motion(msg):
    global turtle_motion
    turtle_motion = msg

def subscribe_turtle_stop(msg):
    global turtle_stop
    turtle_stop = msg.data
    
def subscribe_sonar(msg):
    global height
    height = msg.range


# ================================= GLOBAL FUNCTIONS ==========================================
def is_close_enough(locations, targets, state):
    Dx, Dy, Dz = targets[0] - locations[0], targets[1] - locations[1], targets[2] - locations[2]

    if state == STATE_TAKEOFF:
    	if Dz*Dz <= CLOSE_ENOUGH_SQ:
	    return True
    elif state == STATE_LAND:
    	if Dz*Dz <= CLOSE_ENOUGH_SQ * 2:
	    return True
    else:
    	if Dx*Dx + Dy*Dy <= CLOSE_ENOUGH_SQ:
	    return True
    return False

def too_far(locations, targets, state):
    Dx, Dy, Dz = targets[0] - locations[0], targets[1] - locations[1], targets[2] - locations[2]
    if state == STATE_TAKEOFF or state == STATE_LAND:
    	if Dx*Dx + Dy*Dy + Dz*Dz >= TOO_FAR_SQ:
	    return True
    else:
    	if Dx*Dx + Dy*Dy >= TOO_FAR_SQ:
	    return True
    return False

def next_state(state, turtle_reached):

    if turtle_reached:
	if state == STATE_BASE:
	    return STATE_LAND
	if state == STATE_LAND:
	    return STATE_DONE
	return STATE_BASE
    
    if state == STATE_TAKEOFF:
	return STATE_TURTLE
    if state == STATE_TURTLE:
	return STATE_GOAL
    if state == STATE_GOAL:
	return STATE_TURTLE

    return state

def get_increments(locations, targets):
    Dx, Dy, Dz = targets[0] - locations[0], targets[1] - locations[1], targets[2] - locations[2]
    dist = sqrt(Dx*Dx + Dy*Dy + Dz*Dz)

    if dist > TARGET_SEPARATION:
	Dx, Dy, Dz = TARGET_SEPARATION*Dx/dist, TARGET_SEPARATION*Dy/dist, TARGET_SEPARATION*Dz/dist

    return [Dx, Dy, Dz]


# ================================ BEGIN ===========================================
def master(sx=2., sy=2., gx=2., gy=2.):
    # ---------------------------------- INITS ----------------------------------------------
    # --- init node ---
    rospy.init_node('hector_motion')
    
    # --- cache global vars / constants ---
    global msg_motion, turtle_motion, turtle_stop, height
    msg_motion = None
    turtle_motion = None
    turtle_stop = None
    height = None
    
    
    # --- Publishers ---
    pub_target = rospy.Publisher('/hector/target', PointStamped, latch=True, queue_size=1)
    msg_target = PointStamped()
    msg_target.header.frame_id = "map"
    msg_target_position = msg_target.point
    msg_target_position.x = sx
    msg_target_position.y = sy
    pub_target.publish(msg_target)
    
    pub_state = rospy.Publisher('/hector/state', Int8, latch=True, queue_size=1)
    msg_state = Int8()
    msg_state.data = STATE_TAKEOFF
    pub_state.publish(msg_state)
    
    pub_stop = rospy.Publisher('/hector/stop', Bool, latch=True, queue_size=1)
    msg_stop = Bool()
    msg_stop.data = False
    pub_stop.publish(msg_stop) # ping to others it is ready
    
    # --- Subscribers ---
    rospy.Subscriber('/hector/motion', EE4308MsgMotion, subscribe_motion, queue_size=1)
    rospy.Subscriber('/turtle/motion', EE4308MsgMotion, subscribe_turtle_motion, queue_size=1)
    rospy.Subscriber('/turtle/stop', Bool, subscribe_turtle_stop, queue_size=1)
    rospy.Subscriber('/hector/sonar_height', Range, subscribe_sonar, queue_size=1)
    
    while (height is None or msg_motion is None or turtle_stop is None or \
        turtle_motion is None or rospy.get_time() == 0) and not rospy.is_shutdown():
        pass
    if rospy.is_shutdown():
        return

    # Initial State and Targets
    state = STATE_TAKEOFF
    targets = [sx, sy, TARGET_SEPARATION]
    goals = [sx, sy, CRUISE_ALTITUDE]
    increments = [0, 0, 0.5]
    landing_time = 0
    
    ######################################################
    start = rospy.get_time()
    while rospy.get_time() < start + START_DELAY:
        pass

    t = rospy.get_time()
    while not rospy.is_shutdown():
        if rospy.get_time() > t:
	    # --- Get Current Location ---
	    locations = [msg_motion.x, msg_motion.y, msg_motion.z]

	    # --- Get Next State if Reached Goal ---
	    if is_close_enough(locations, goals, state):
		state = next_state(state, turtle_stop)
		
		if state == STATE_DONE:
		    print("GOAL REACHED")
		    break
		elif state == STATE_LAND:
		    goals = [sx, sy, 0]
		    landing_time = rospy.get_time()
	        elif state == STATE_TURTLE:
		    goals = [turtle_motion.x, turtle_motion.y, CRUISE_ALTITUDE]
	        elif state == STATE_GOAL:
		    goals = [gx, gy, CRUISE_ALTITUDE]
	        else:
		    goals = [sx, sy, CRUISE_ALTITUDE]
		
	    elif state == STATE_TURTLE: # Constantly update the goal in STATE_TURTLE as turtle bot is not static
		goals = [turtle_motion.x, turtle_motion.y, CRUISE_ALTITUDE]

            # --- Publish state ---
            msg_state.data = state
            pub_state.publish(msg_state)

	    # --- Get incremental targets ---
	    if is_close_enough(locations, targets, state):
		increments = get_increments(locations, goals)
		targets = [locations[0] + increments[0], locations[1] + increments[1], locations[2] + increments[2]]
	    
	    # --- Re-generate targets if the hector moves too far from its current target ---
	    if too_far(locations, targets, state):
		increments = get_increments(locations, goals)
		targets = [locations[0] + increments[0], locations[1] + increments[1], locations[2] + increments[2]]
	    
	    print(state)
	    print(targets)
	    print(goals)
            
            # --- Publish target ---
            msg_target_position.x = targets[0]
            msg_target_position.y = targets[1]
            msg_target_position.z = targets[2]
            msg_target.header.seq += 1
            pub_target.publish(msg_target)

	    if state == STATE_LAND and landing_time + 7 < rospy.get_time():
		print("GOAL REACHED")
		break
            
            # --- Timing ---
            et = rospy.get_time() - t
            t += ITERATION_PERIOD
            if et > ITERATION_PERIOD:
                print('[HEC MASTER] {} OVERSHOOT'.format(int(et*1000)))
                
  
    # --- Publish stop ---
    msg_stop.data = True
    pub_stop.publish(msg_stop) # ping to others it is ready
    ######################################################
    
if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            goals = sys.argv[3]
            goals = goals.split('|')
            goals = goals[-1]
            goals = goals.split(',')
            gx = float(goals[0]); gy = float(goals[1])
            master(float(sys.argv[1]), float(sys.argv[2]), gx, gy)
        else:
            master()
    except rospy.ROSInterruptException:
        pass
        
    print('=== [HEC MASTER] Terminated ===')
