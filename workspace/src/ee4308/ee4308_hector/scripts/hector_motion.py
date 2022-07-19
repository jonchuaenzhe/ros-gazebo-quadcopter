#!/usr/bin/env python

import roslib, rospy, rospkg
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Int8, Bool
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from hector_uav_msgs.msg import Altimeter
from std_srvs.srv import Empty # calibrate imu, probably not needed
from math import sqrt, cos, sin, pi, atan2
from numpy import array, transpose, matmul
from numpy.linalg import inv
import sys
import json

from hector_constants import *
from ee4308_bringup.msg import EE4308MsgMotion, EE4308MsgMaster

# ================================= PARAMETERS ========================================== 
ITERATION_PERIOD = 0.01

# ================================= GENERAL FUNCTIONS ==========================================
def rad_from_deg(deg):
    return (deg/180) * pi

# ================================= GPS FUNCTIONS ==========================================
def N(phi):
    e_sq = 1 - (RAD_POLAR * RAD_POLAR / (RAD_EQUATOR * RAD_EQUATOR))
    denom = sqrt(1 - e_sq * sin(phi) * sin(phi))
    return RAD_EQUATOR/denom

def R_e_n(phi, l):
    row1 = [-sin(phi) * cos(l), -sin(l), -cos(phi) * cos(l)]
    row2 = [-sin(phi) * sin(l), cos(l), -cos(phi) * sin(l)]
    row3 = [cos(phi), 0, -sin(phi)]
    return array([row1, row2, row3])

def R_m_n(phi_0):
    row1 = [cos(phi_0), sin(phi_0), 0]
    row2 = [sin(phi_0), -cos(phi_0), 0]
    row3 = [0, 0, -1]
    return array([row1, row2, row3])

def ecef_from_geodetic(phi, l, h): # phi and l in radians, returns a numpy array of ecef coordinates
    x = (N(phi) + h) * cos(phi) * cos(l)
    y = (N(phi) + h) * cos(phi) * sin(l)
    z = (((RAD_POLAR * RAD_POLAR) / (RAD_EQUATOR * RAD_EQUATOR)) * N(phi) + h) * sin(phi)
    return array([x, y, z])

def map_from_gps(gps, initial_ecef, initial_map): # takes in 1 array, 2 numpy arrays
    ecef = ecef_from_geodetic(gps[0], gps[1], gps[2])
    ned = matmul(R_e_n(gps[0], gps[1]).T, ecef - initial_ecef)
    return matmul(R_m_n(0), ned) + initial_map

# ================================= EKF FUNCTIONS ==========================================

# POSE CONSTANTS
F_p = array([[1, ITERATION_PERIOD], [0, 1]])
W_p = array([[0.5 * ITERATION_PERIOD * ITERATION_PERIOD], [ITERATION_PERIOD]])
H_p = array([[1, 0]])

# ORIENTATION CONSTANTS
F_o = array([[1]])
W_o = array([[ITERATION_PERIOD]])
H_o = array([[1]])

# FUNCTIONS FOR SINGLE CHANNEL
def prediction(pose, cov, F, W, inp, q):
    pose = matmul(F, pose) + matmul(W, array([[inp]]))
    cov = matmul(F, matmul(cov, F.T)) + matmul(W, matmul(array([[q]]), W.T))
    return pose, cov

def correction(pose, cov, H, meas, r):
    K = matmul(cov, H.T) / (matmul(H, matmul(cov, H.T)) + array([[r]]))[0][0]
    pose = pose + K * (meas - pose[0][0])
    cov = cov - matmul(K, matmul(H, cov))
    return pose, cov

# =============================== SUBSCRIBERS =========================================  
rbt_true = [False, False, False, False]
def subscribe_true(msg):
    tmp = msg.pose.position
    rbt_true[0] = tmp.x
    rbt_true[1] = tmp.y
    rbt_true[2] = tmp.z
    tmp = msg.pose.orientation
    rbt_true[3] = euler_from_quaternion([tmp.x, tmp.y, tmp.z, tmp.w])[2]

# GPS -> MEASUREMENT
rbt_gps = [False, False, False, False]
def subscribe_gps(msg):
    rbt_gps[0] = msg.latitude # phi
    rbt_gps[1] = msg.longitude # l
    rbt_gps[2] = msg.altitude # h
    rbt_gps[3] = msg.header.seq

# IMU -> PREDICTION
rbt_imu = [False, False, False, False, False]
def subscribe_imu(msg):
    tmp = msg.linear_acceleration
    rbt_imu[0] = tmp.x
    rbt_imu[1] = tmp.y
    rbt_imu[2] = tmp.z
    rbt_imu[3] = msg.angular_velocity.z
    rbt_imu[4] = msg.header.seq

# MAGNETOMETER -> MEASUREMENT
rbt_magnet = [False, False, False]
def subscribe_magnetic(msg):
    tmp = msg.vector
    rbt_magnet[0] = tmp.x
    rbt_magnet[1] = tmp.y
    rbt_magnet[2] = msg.header.seq

# BAROMETER/ALTIMETER -> MEASUREMENT
rbt_alt = [False, False]
def subscribe_altimeter(msg):
    rbt_alt[0] = msg.altitude
    rbt_alt[1] = msg.header.seq

# STATE DATA
def subscribe_state(msg):
    global msg_state
    msg_state = msg.data

def subscribe_stop(msg):
    global msg_stop
    msg_stop = msg.data
    

# ================================ BEGIN ===========================================
def motion(rx0=2.0, ry0=2.0, rz0=0.1825, ro0=0.0):
    rz0=0.1825
    # ---------------------------------- INITS ----------------------------------------------
    # --- init node ---
    rospy.init_node('hector_motion')
    
    # --- cache global vars / constants ---
    global msg_stop
    msg_stop = None
    
    # --- Service: Calibrate ---
    calibrate_imu = rospy.ServiceProxy('/hector/raw_imu/calibrate', Empty)
    calibrate_imu()
    print('[HEC MOTION] Imu calibrated')
    
    # --- Publishers ---
    pub_motion = rospy.Publisher('/hector/motion', EE4308MsgMotion, latch=True, queue_size=1)
    msg_motion = EE4308MsgMotion()
    pub_motion.publish(msg_motion)
    
    pub_pose = rospy.Publisher('/hector/motion_pose', PoseStamped, latch=True, queue_size=1) # for rviz
    msg_pose = PoseStamped()
    msg_pose.header.frame_id = "map"
    msg_pose_position = msg_pose.pose.position
    msg_pose_orientation = msg_pose.pose.orientation
    
    # --- Subscribers ---
    rospy.Subscriber('/hector/ground_truth_to_tf/pose', PoseStamped, subscribe_true, queue_size=1)
    rospy.Subscriber('/hector/fix', NavSatFix, subscribe_gps, queue_size=1)
    rospy.Subscriber('/hector/raw_imu', Imu, subscribe_imu, queue_size=1)
    rospy.Subscriber('/hector/magnetic', Vector3Stamped, subscribe_magnetic, queue_size=1)
    rospy.Subscriber('/hector/altimeter', Altimeter, subscribe_altimeter, queue_size=1)
    rospy.Subscriber('/hector/state', Int8, subscribe_state, queue_size=1)
    rospy.Subscriber('/hector/stop', Bool, subscribe_stop, queue_size=1)
    
    while (not rbt_true[-1] or not rbt_imu[-1] or not rbt_gps[-1] or not rbt_magnet[-1] or\
        rospy.get_time() == 0 or not rbt_alt[-1] or msg_stop is None) and not rospy.is_shutdown():
        pass

    if rospy.is_shutdown():
        return
         
    print('=== [HEC MOTION] Initialised ===')
    
    # ---------------------------------- LOOP ----------------------------------------------
    if USE_GROUND_TRUTH:
	
        t = rospy.get_time()
        while not rospy.is_shutdown() and not msg_stop:
            if rospy.get_time() > t:
		
                # publish true motion
                msg_motion.x = rbt_true[0]
                msg_motion.y = rbt_true[1]
                msg_motion.z = rbt_true[2]
                msg_motion.o = rbt_true[3]
                pub_motion.publish(msg_motion)
                
                # publish results to rviz
                msg_pose_position.x = msg_motion.x
                msg_pose_position.y = msg_motion.y
                msg_pose_position.z = msg_motion.z
                tmp = quaternion_from_euler(0, 0, msg_motion.o)
                msg_pose_orientation.x = tmp[0]
                msg_pose_orientation.y = tmp[1]
                msg_pose_orientation.z = tmp[2]
                msg_pose_orientation.w = tmp[3]
                pub_pose.publish(msg_pose)
                
                # iterate
                t += ITERATION_PERIOD

    else:
        # Initialise Loactions (named as pose) and their Covariances
	pose_x, pose_y, pose_z = array([[rx0], [0]]), array([[ry0], [0]]), array([[rz0], [0]])
	cov_x, cov_y, cov_z = array([[0, 0], [0, 0]]), array([[0, 0], [0, 0]]), array([[0, 0], [0, 0]])

	# Initialise Orientation and its Covariance
	orient = array([[ro0]])
	cov_orient = array([[0]])
	
	# Keep track of whenever new readings of the measurements come in
	imu_seq, gps_seq, alt_seq, mag_seq = 0, 0, 0, 0
	
	# Calibration of initial ECEF and Altitude by averaging multiple samples
	ecef_x_ls, ecef_y_ls, ecef_z_ls = [], [], []
	alt_ls = []
	start = rospy.get_time()
	while rospy.get_time() < start + START_DELAY:
	    if rbt_gps[3] > gps_seq:
	    	ecef = ecef_from_geodetic(rad_from_deg(rbt_gps[0]), rad_from_deg(rbt_gps[1]), rbt_gps[2])
	    	ecef_x_ls.append(ecef[0])
	    	ecef_y_ls.append(ecef[1])
	    	ecef_z_ls.append(ecef[2])
		gps_seq = rbt_gps[3]

	    if rbt_alt[1] > alt_seq:
 	    	alt_ls.append(rbt_alt[0])
		alt_seq = rbt_alt[1]
	
	num_pts = len(ecef_x_ls)
	initial_ecef = array([sum(ecef_x_ls)/num_pts, sum(ecef_y_ls)/num_pts, sum(ecef_z_ls)/num_pts])
	initial_alt = sum(alt_ls)/len(alt_ls)

	initial_map = array([rx0, ry0, rz0])
        
        # --- Loop EKF ---
        t = rospy.get_time()
        while not rospy.is_shutdown() and not msg_stop:
            if rospy.get_time() > t:
		phi, l, h = rad_from_deg(rbt_gps[0]), rad_from_deg(rbt_gps[1]), rbt_gps[2]
		gps_arr = map_from_gps([phi, l, h], initial_ecef, initial_map)
		gps_arr = [gps_arr[0], gps_arr[1], gps_arr[2]]
		print("Map Coordinates:")
		print(gps_arr)
		imu_arr = [rbt_imu[0], rbt_imu[1], rbt_imu[2], rbt_imu[3]]
		print("IMU Readings:")
		print(imu_arr)
		mag = -(atan2(rbt_magnet[1], rbt_magnet[0]))
		print("magnetic angle:")
		print(mag)
		alt = rbt_alt[0] - initial_alt + rz0
		print("altitude:")
		print(alt)
		
		# Prediction Step with IMU
		if rbt_imu[4] > imu_seq:
		    imu_x = cos(orient[0][0]) * (-rbt_imu[0]) - sin(orient[0][0]) * (-rbt_imu[1])
		    imu_y = cos(orient[0][0]) * (-rbt_imu[1]) + sin(orient[0][0]) * (-rbt_imu[0])

		    pose_x, cov_x = prediction(pose_x, cov_x, F_p, W_p, imu_x, IMU_NX)
		    pose_y, cov_y = prediction(pose_y, cov_y, F_p, W_p, imu_y, IMU_NY)
		    pose_z, cov_z = prediction(pose_z, cov_z, F_p, W_p, rbt_imu[2] - G, IMU_NZ)

		    orient, cov_orient = prediction(orient, cov_orient, F_o, W_o, rbt_imu[3], IMU_NO)

		    imu_seq = rbt_imu[4]
		
		# Correction Step with GPS
		if rbt_gps[3] > gps_seq:
		    meas = map_from_gps([phi, l, h], initial_ecef, initial_map)
		    pose_x, cov_x = correction(pose_x, cov_x, H_p, meas[0], GPS_NX)
		    pose_y, cov_y = correction(pose_y, cov_y, H_p, meas[1], GPS_NY)
		    pose_z, cov_z = correction(pose_z, cov_z, H_p, meas[2], GPS_NZ)

		    gps_seq = rbt_gps[3]

		# Correction Step with Altimeter
		if rbt_alt[1] > alt_seq:
		    meas = rbt_alt[0] - initial_alt + rz0
		    if msg_state == STATE_TAKEOFF:
		   	pose_z, cov_z = correction(pose_z, cov_z, H_p, meas, 0.01)
		    else:
		    	pose_z, cov_z = correction(pose_z, cov_z, H_p, meas, ALT_NZ)

		    alt_seq = rbt_alt[1]
		
		# Correction step with magnetometer
		if rbt_magnet[2] > mag_seq:
		    mag = -(atan2(rbt_magnet[1], rbt_magnet[0]))
		    
		    # add/subtract 2pi from magnetometer angle if discontinuity issue occurs
		    if orient[0][0] <= pi and orient[0][0] > pi*3/4 and mag >= -pi and mag < -pi*3/4:
			mag = mag + 2*pi
		    if orient[0][0] >= -pi and orient[0][0] < -pi*3/4 and mag <= pi and mag > pi*3/4:
			mag = mag - 2*pi
			
		    orient, cov_orient = correction(orient, cov_orient, H_o, mag, MAG_NO)

		    mag_seq = rbt_magnet[2]
		
		# Correction for discontinuity
		if orient[0][0] > pi:
		    orient[0][0] = orient[0][0] - 2*pi
		if orient[0][0] < -pi:
		    orient[0][0] = orient[0][0] + 2*pi
		
		print("Calculated:")
		print([pose_x[0][0], pose_y[0][0], pose_z[0][0], orient[0][0]])
		print("True:")
		print(rbt_true)
                
                # --- Publish motion ---
                msg_motion.x = pose_x[0][0] # m
                msg_motion.y = pose_y[0][0] # m
                msg_motion.z = pose_z[0][0] # m
                msg_motion.o = orient[0][0] # orientation in z (rad)
                pub_motion.publish(msg_motion)
                
                # publish results to rviz
                msg_pose_position.x = msg_motion.x
                msg_pose_position.y = msg_motion.y
                msg_pose_position.z = msg_motion.z
                tmp = quaternion_from_euler(0, 0, msg_motion.o)
                msg_pose_orientation.x = tmp[0]
                msg_pose_orientation.y = tmp[1]
                msg_pose_orientation.z = tmp[2]
                msg_pose_orientation.w = tmp[3]
                pub_pose.publish(msg_pose)
                
                # --- Iterate ---
                et = rospy.get_time() - t
                t += ITERATION_PERIOD
                if et > ITERATION_PERIOD:
                    print('[HEC MOTION] {}ms OVERSHOOT'.format(int(et*1000)))
                    
                    
        ##########################################################
        
if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            motion(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), 0.0)
        else:
            motion()
    except rospy.ROSInterruptException:
        pass
        
    print('=== [HEC MOTION] Terminated ===')
