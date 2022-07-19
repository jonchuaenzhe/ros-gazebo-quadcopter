#!/usr/bin/env python
import roslib, rospy, rospkg
from math import sqrt, cos, sin, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy import inf
import sys
from turtle_constants import TWOPI, AXLE_TRACK, WHEEL_RADIUS

# import messages
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from ee4308_bringup.msg import EE4308MsgMotion

# ================================= PARAMETERS ========================================== 
ITERATION_PERIOD = 0.02 #s

# ================================= CONSTANTS ==========================================

# =============================== SUBSCRIBERS =========================================  
def subscribe_true(msg):
    # subscribes to the robot's true position in the simulator. This should not be used, for checking only.
    tmp = msg.transforms[0]
    if tmp.header.frame_id != "turtle/odom":
        return
    msg_tf = tmp.transform
    
    # the following can be done from topic /turtle/odom  as well. in fact the data here is the same as that of turtle/odom
    global rbt_true_o, rbt_true_x, rbt_true_y # write to global variables
    rbt_true_x = msg_tf.translation.x
    rbt_true_y = msg_tf.translation.y
    rbt_true_o = euler_from_quaternion([\
            msg_tf.rotation.x, \
            msg_tf.rotation.y, \
            msg_tf.rotation.z, \
            msg_tf.rotation.w, \
        ])[2]
        
def subscribe_wheels(msg):
    global rbt_wheel_l, rbt_wheel_r
    rbt_wheel_l = msg.position[1]
    rbt_wheel_r = msg.position[0]

def subscribe_imu(msg):
    global rbt_imu_o, rbt_imu_w, rbt_imu_a # write to these global variables
    t = msg.orientation
    # imu_o is calculated internally in imu as ~ imu_w*dt
    rbt_imu_o = euler_from_quaternion([\
        t.x,\
        t.y,\
        t.z,\
        t.w\
        ])[2]
    rbt_imu_w = msg.angular_velocity.z
    rbt_imu_a = msg.linear_acceleration.x
     
def subscribe_stop(msg):
    global stop
    stop = msg.data
   
def limit_angle(angle):
    return (angle + pi) % TWOPI - pi
    
# ================================ BEGIN ===========================================
def motion(filter_x=-2., filter_y=-.5, filter_o=.0):
    
    # init node
    rospy.init_node('turtle_motion')
    
    # Initialise last global var written in each subscribe handler so we know when all first messages are parsed
    global rbt_true_o, rbt_wheel_r, rbt_imu_a, stop 
    rbt_true_o = inf
    rbt_wheel_r = inf
    rbt_imu_a = inf
    stop = None
       
    # Subscribers
    rospy.Subscriber('/tf', TFMessage, subscribe_true, queue_size=1) # broadcast by tf tree, which is true position. identical to /turtle/odom only in sim.
    rospy.Subscriber('/turtle/joint_states', JointState, subscribe_wheels, queue_size=1)
    rospy.Subscriber('/turtle/imu', Imu, subscribe_imu, queue_size=1)
    rospy.Subscriber('/turtle/stop', Bool, subscribe_stop, queue_size=1)
    
    # Publishers
    pub_motion = rospy.Publisher('/turtle/motion', EE4308MsgMotion, latch=True, queue_size=1)
    pub_pose = rospy.Publisher('/turtle/motion_pose', PoseStamped, latch=True, queue_size=1) # for rviz
    
    # Wait for Subscribers to receive data.
    # bypass this loop if shutdown command is received
    while (stop is None or rbt_true_o == inf or rbt_wheel_r == inf or rbt_imu_a == inf or\
        rospy.get_time() == 0) and not rospy.is_shutdown():
        pass
        
    # if shutdown received, quit
    if rospy.is_shutdown():
        return
    
    # inits
    filter_v = .0 # initial forward velocity (m/s)
    filter_w = .0 # initial angular velocity (rad/s)
    # filter_o # initial value depends on function parameter: heading / angle (rad)
    # filter_x # initial value depends on function parameter: position in x (m)
    # filter_y # initial value depends on function parameter: position in y (m)
    prev_filter_o = filter_o
    prev_imu_o = rbt_imu_o
    prev_wheel_l = rbt_wheel_l
    prev_wheel_r = rbt_wheel_r
    
    # init /turtle/motion_pose, for Rviz
    msg_pose = PoseStamped()
    msg_pose.header.frame_id = "map"
    msg_pose_position = msg_pose.pose.position
    msg_pose_orientation = msg_pose.pose.orientation
    msg_pose_position.z = 0.18 # purely for visualisation only.
    
    # send a message to the /turtle/motion topic
    msg_motion = EE4308MsgMotion() # instantiate a Motion message class that we use throughout the program
    msg_motion.x = filter_x
    msg_motion.y = filter_y
    msg_motion.o = filter_o
    msg_motion.v = filter_v
    msg_motion.w = filter_w        
    pub_motion.publish(msg_motion)
    
    print('=== [TTL MOTION] Initialised ===')
    
    # get simulated time
    prev_t = rospy.get_time();
    t_iter = prev_t # time counter to make sure loops iterate (at least) every ITERATION_PERIOD seconds
    
    # use /turtle/odom (via /tf). So x,y,o are ground truths in simulation. v and w are estimates here.
    while not rospy.is_shutdown() and not stop:
        # loop until shutdown is requested or master sends stop
        
        if rospy.get_time() > t_iter: 
            # make sure loops iterate (at least) every ITERATION_PERIOD seconds
        
            dt = rospy.get_time() - prev_t # get the time increment
            if dt <= 0.:
                print("[TTL MOTION] dt = {} <= 0 detected. Likely due to overshoot. Skip. prev_t = {}".format(dt, prev_t))
                continue
                
            dwl = rbt_wheel_l - prev_wheel_l
            dwr = rbt_wheel_r - prev_wheel_r
            
            # =================== FILTER: ODOMETRY MOTION MODEL FUSED WITH IMU & ENCODER) ====================
            dwm = dwr - dwl
            dws = dwr + dwl
            
            # get new filtered forward velocity
            filter_v = WHEEL_RADIUS * dws / 2 / dt
            
            # get new filtered angular velocity
            filter_w = rbt_imu_w
                
            # publish filter results
            msg_motion.x = rbt_true_x # from /turtle/odom. is ground truth in simulation 
            msg_motion.y = rbt_true_y # from /turtle/odom. is ground truth in simulation 
            msg_motion.o = rbt_true_o # from /turtle/odom. is ground truth in simulation 
            msg_motion.v = filter_v # is an estimate. Not used anywhere else
            msg_motion.w = filter_w # is an estimate. Not used anywhere else
            pub_motion.publish(msg_motion) #topic turtle/motion
            
            # publish results to rviz
            msg_pose_position.x = msg_motion.x
            msg_pose_position.y = msg_motion.y
            tmp = quaternion_from_euler(0, 0, msg_motion.o)
            msg_pose_orientation.x = tmp[0]
            msg_pose_orientation.y = tmp[1]
            msg_pose_orientation.z = tmp[2]
            msg_pose_orientation.w = tmp[3]
            pub_pose.publish(msg_pose)
            
            # check if within loop timing
            t_error = (rospy.get_time() - t_iter) # must be less than ITERATION_PERIOD so every loop is within ITERATION_PERIOD
            if t_error > ITERATION_PERIOD:
                print('[TTL MOTION] {}ms OVERSHOOT'.format(int(t_error*1000)))
            # update next loop's starting time
            t_iter += ITERATION_PERIOD
            
    
    
if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            motion(float(sys.argv[1]), float(sys.argv[2]), .0)
        else:
            motion()
    except rospy.ROSInterruptException:
        pass
        
    # terminated
    print('=== [TTL MOTION] Terminated ===')
