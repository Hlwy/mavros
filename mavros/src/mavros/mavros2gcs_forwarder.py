#!/usr/bin/env python
from __future__ import print_function

import math
import signal
import threading
import rospy, tf
import numpy as np
import os, csv, time, sys
from dronekit import connect

import mavros
from mavros.utils import *
from mavros.mavlink import *
from mavros_msgs.msg import *

os.environ["MAVLINK20"] = "1"       # Set MAVLink protocol to 2.
sys.path.append("/usr/local/lib/")

# Replacement of the standard print() function to flush the output
def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

"""
SNIPPET - REFERENCE
might be good reference for checking logic for vision_position_delta's glitch detection

# Check for pose jump and increment reset_counter
if prev_data != None:
    delta_translation = [data.translation.x - prev_data.translation.x, data.translation.y - prev_data.translation.y, data.translation.z - prev_data.translation.z]
    delta_velocity = [data.velocity.x - prev_data.velocity.x, data.velocity.y - prev_data.velocity.y, data.velocity.z - prev_data.velocity.z]
    position_displacement = np.linalg.norm(delta_translation)
    speed_delta = np.linalg.norm(delta_velocity)

    # Pose jump is indicated when position changes abruptly. The behavior is not well documented yet (as of librealsense 2.34.0)
    jump_threshold = 0.1 # in meters, from trials and errors, should be relative to how frequent is the position data obtained (200Hz for the T265)
    jump_speed_threshold = 20.0 # in m/s from trials and errors, should be relative to how frequent is the velocity data obtained (200Hz for the T265)
    if (position_displacement > jump_threshold) or (speed_delta > jump_speed_threshold):
        send_msg_to_gcs('VISO jump detected')
        if position_displacement > jump_threshold:
            progress("Position jumped by: %s" % position_displacement)
        elif speed_delta > jump_speed_threshold:
            progress("Speed jumped by: %s" % speed_delta)
            increment_reset_counter()
"""
class Mavros2GcsForwarder:
    # lock for thread synchronization
    lock = threading.Lock()
    mavlink_thread_should_exit = False

    # Default configurations for connection to the FCU
    connection_string_default = '/dev/ttyUSB0'
    connection_baudrate_default = 921600
    connection_timeout_sec_default = 5

    def __init__(self):
        rospy.init_node('mavros2gcs_forwarder_noder')
        self.ns = rospy.get_namespace()

        connection_string = args.connect
        connection_baudrate = args.baudrate
        # Using default values if no specified inputs
        if not connection_string:
            connection_string = connection_string_default
            progress("INFO: Using default connection_string %s" % connection_string)
        else: progress("INFO: Using connection_string %s" % connection_string)

        if not connection_baudrate:
            connection_baudrate = connection_baudrate_default
            progress("INFO: Using default connection_baudrate %s" % connection_baudrate)
        else: progress("INFO: Using connection_baudrate %s" % connection_baudrate)

        if not debug_enable: debug_enable = 0
        else:
            debug_enable = 1
            np.set_printoptions(precision=4, suppress=True) # Format output on terminal
            progress("INFO: Debug messages enabled.")




        mavlink_in_topic = rospy.get_param('~mavlink_input_topic', "/mavros/from")
        speed_topic = rospy.get_param('~speed_topic', "mavros/vision_speed/speed_twist_cov")

        rospy.Subscriber(vo_topic, Odometry, self.voCallback)
        rospy.Subscriber(pose_topic, PoseStamped, self.poseCallback)

        self.out_pub = rospy.Publisher(output_topic, PoseWithCovarianceStamped, queue_size=10)
        self.reset_srv = rospy.Service("vision_pose_republisher/reset", Empty, self.reset_vo)

        self.r = rospy.Rate(update_rate)
        rospy.loginfo("mavros_vision_pose_republisher Node --- Started...")

    def mavlinkInputCallback(self, msg):
        if(not self.recvd_initial_pose):
            data =  msg.pose
            (x,y,z,r,p,yaw) = quat2euler(data)
            self.pose_offset = [x,y,z,r,p,yaw]
            self.recvd_initial_pose = True
            print("[INFO] mavros_vision_pose_republisher Node --- Recieved Initial Pose = %.4f, %.4f, %.4f - At - %.4f, %.4f, %.4f"
                % (x,y,z,np.rad2deg(r), np.rad2deg(p), np.rad2deg(yaw))
            )
            self.allow_msg_pub = True



#######################################
# Parameters
#######################################





# default exit code is failure - a graceful termination with a
# terminate signal is possible.
exit_code = 1


#######################################
# Global variables
#######################################

# Data variables
data = None
prev_data = None
current_time_us = 0

# Increment everytime pose_jumping or relocalization happens
# See here: https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#are-there-any-t265-specific-options
# For AP, a non-zero "reset_counter" would mean that we could be sure that the user's setup was using mavlink2
reset_counter = 1

#######################################
# Parsing user' inputs
#######################################



#######################################
# Functions - MAVLink
#######################################

def mavlink_loop(conn, callbacks):
    '''a main routine for a thread; reads data from a mavlink connection,
    calling callbacks based on message type received.
    '''
    interesting_messages = list(callbacks.keys())
    while not mavlink_thread_should_exit:
        # send a heartbeat msg
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0
        )
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None: continue
        callbacks[m.get_type()](m)

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_estimate_message():
    global current_time_us, H_aeroRef_aeroBody, reset_counter
    with lock:
        if H_aeroRef_aeroBody is not None:
            # Setup angle data
            rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))

            # Setup covariance data, which is the upper right triangle of the covariance matrix, see here: https://files.gitter.im/ArduPilot/VisionProjects/1DpU/image.png
            # Attemp #01: following this formula https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/src/base_realsense_node.cpp#L1406-L1411
            cov_pose    = linear_accel_cov * pow(10, 3 - int(data.tracker_confidence))
            cov_twist   = angular_vel_cov  * pow(10, 1 - int(data.tracker_confidence))
            covariance  = np.array([cov_pose, 0, 0, 0, 0, 0,
                                       cov_pose, 0, 0, 0, 0,
                                          cov_pose, 0, 0, 0,
                                            cov_twist, 0, 0,
                                               cov_twist, 0,
                                                  cov_twist])

            # Send the message
            conn.mav.vision_position_estimate_send(
                current_time_us,            # us Timestamp (UNIX time or time since system boot)
                H_aeroRef_aeroBody[0][3],   # Global X position
                H_aeroRef_aeroBody[1][3],   # Global Y position
                H_aeroRef_aeroBody[2][3],   # Global Z position
                rpy_rad[0],	                # Roll angle
                rpy_rad[1],	                # Pitch angle
                rpy_rad[2],	                # Yaw angle
                covariance,                 # Row-major representation of pose 6x6 cross-covariance matrix
                reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
            )

# https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
def send_vision_position_delta_message():
    global current_time_us, current_confidence_level, H_aeroRef_aeroBody
    with lock:
        if H_aeroRef_aeroBody is not None:
            # Calculate the deltas in position, attitude and time from the previous to current orientation
            H_aeroRef_PrevAeroBody      = send_vision_position_delta_message.H_aeroRef_PrevAeroBody
            H_PrevAeroBody_CurrAeroBody = (np.linalg.inv(H_aeroRef_PrevAeroBody)).dot(H_aeroRef_aeroBody)

            delta_time_us    = current_time_us - send_vision_position_delta_message.prev_time_us
            delta_position_m = [H_PrevAeroBody_CurrAeroBody[0][3], H_PrevAeroBody_CurrAeroBody[1][3], H_PrevAeroBody_CurrAeroBody[2][3]]
            delta_angle_rad  = np.array( tf.euler_from_matrix(H_PrevAeroBody_CurrAeroBody, 'sxyz'))

            # Send the message
            conn.mav.vision_position_delta_send(
                current_time_us,    # us: Timestamp (UNIX time or time since system boot)
                delta_time_us,	    # us: Time since last reported camera frame
                delta_angle_rad,    # float[3] in radian: Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientation
                delta_position_m,   # float[3] in m: Change in position from previous to current frame rotated into body frame (0=forward, 1=right, 2=down)
                current_confidence_level # Normalized confidence value from 0 to 100.
            )

            # Save static variables
            send_vision_position_delta_message.H_aeroRef_PrevAeroBody = H_aeroRef_aeroBody
            send_vision_position_delta_message.prev_time_us = current_time_us

# https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
def send_vision_speed_estimate_message():
    global current_time_us, V_aeroRef_aeroBody, reset_counter
    with lock:
        if V_aeroRef_aeroBody is not None:

            # Attemp #01: following this formula https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/src/base_realsense_node.cpp#L1406-L1411
            cov_pose    = linear_accel_cov * pow(10, 3 - int(data.tracker_confidence))
            covariance  = np.array([cov_pose,   0,          0,
                                    0,          cov_pose,   0,
                                    0,          0,          cov_pose])

            # Send the message
            conn.mav.vision_speed_estimate_send(
                current_time_us,            # us Timestamp (UNIX time or time since system boot)
                V_aeroRef_aeroBody[0][3],   # Global X speed
                V_aeroRef_aeroBody[1][3],   # Global Y speed
                V_aeroRef_aeroBody[2][3],   # Global Z speed
                covariance,                 # covariance
                reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
            )

# Update the changes of confidence level on GCS and terminal
def update_tracking_confidence_to_gcs():
    if data is not None and update_tracking_confidence_to_gcs.prev_confidence_level != data.tracker_confidence:
        confidence_status_string = 'Tracking confidence: ' + pose_data_confidence_level[data.tracker_confidence]
        send_msg_to_gcs(confidence_status_string)
        update_tracking_confidence_to_gcs.prev_confidence_level = data.tracker_confidence

# https://mavlink.io/en/messages/common.html#STATUSTEXT
def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'T265: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)


# Send a mavlink SET_GPS_GLOBAL_ORIGIN message (http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN), which allows us to use local position information without a GPS.
def set_default_global_origin():
    conn.mav.set_gps_global_origin_send(
        1,
        home_lat,
        home_lon,
        home_alt
    )

# Send a mavlink SET_HOME_POSITION message (http://mavlink.org/messages/common#SET_HOME_POSITION), which allows us to use local position information without a GPS.
def set_default_home_position():
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    conn.mav.set_home_position_send(
        1,
        home_lat,
        home_lon,
        home_alt,
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
    )


# Request a timesync update from the flight controller, for future work.
# TODO: Inspect the usage of timesync_update
def update_timesync(ts=0, tc=0):
    if ts == 0:
        ts = int(round(time.time() * 1000))
    conn.mav.timesync_send(
        tc,     # tc1
        ts      # ts1
    )
#######################################
# Functions - Miscellaneous
#######################################

# Monitor user input from the terminal and perform action accordingly
def user_input_monitor():
    global scale_factor
    while True:
        # Special case: updating scale
        if scale_calib_enable == True:
            scale_factor = float(input("INFO: Type in new scale as float number\n"))
            progress("INFO: New scale is %s" % scale_factor)

        if enable_auto_set_ekf_home:
            send_msg_to_gcs('Set EKF home with default GPS location')
            set_default_global_origin()
            set_default_home_position()
            time.sleep(1) # Wait a short while for FCU to start working

        # Add new action here according to the key pressed.
        # Enter: Set EKF home when user press enter
        try:
            c = input()
            if c == "":
                send_msg_to_gcs('Set EKF home with default GPS location')
                set_default_global_origin()
                set_default_home_position()
            else:
                progress("Got keyboard input %s" % c)
        except IOError: pass


#######################################
# Main code starts here
#######################################

progress("INFO: Starting Vehicle communications")
conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect = True,
    source_system = 1,
    source_component = 93,
    baud=connection_baudrate,
    force_connected=True,
)

mavlink_callbacks = {
    'ATTITUDE': att_msg_callback,
}

mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

# connecting and configuring the camera is a little hit-and-miss.
# Start a timer and rely on a restart of the script to get it working.
# Configuring the camera appears to block all threads, so we can't do
# this internally.

# send_msg_to_gcs('Setting timer...')
signal.setitimer(signal.ITIMER_REAL, 5)  # seconds...

send_msg_to_gcs('Connecting to camera...')
realsense_connect()
send_msg_to_gcs('Camera connected.')

signal.setitimer(signal.ITIMER_REAL, 0)  # cancel alarm

# Send MAVlink messages in the background at pre-determined frequencies
sched = BackgroundScheduler()

if enable_msg_vision_position_estimate:
    sched.add_job(send_vision_position_estimate_message, 'interval', seconds = 1/vision_position_estimate_msg_hz)

if enable_msg_vision_position_delta:
    sched.add_job(send_vision_position_delta_message, 'interval', seconds = 1/vision_position_delta_msg_hz)
    send_vision_position_delta_message.H_aeroRef_PrevAeroBody = tf.quaternion_matrix([1,0,0,0])
    send_vision_position_delta_message.prev_time_us = int(round(time.time() * 1000000))

if enable_msg_vision_speed_estimate:
    sched.add_job(send_vision_speed_estimate_message, 'interval', seconds = 1/vision_speed_estimate_msg_hz)

if enable_update_tracking_confidence_to_gcs:
    sched.add_job(update_tracking_confidence_to_gcs, 'interval', seconds = 1/update_tracking_confidence_to_gcs_hz_default)
    update_tracking_confidence_to_gcs.prev_confidence_level = -1

# A separate thread to monitor user input
if enable_user_keyboard_input:
    user_keyboard_input_thread = threading.Thread(target=user_input_monitor)
    user_keyboard_input_thread.daemon = True
    user_keyboard_input_thread.start()
    progress("INFO: Press Enter to set EKF home at default location")

sched.start()

# gracefully terminate the script if an interrupt signal (e.g. ctrl-c)
# is received.  This is considered to be abnormal termination.
main_loop_should_quit = False
def sigint_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True
signal.signal(signal.SIGINT, sigint_handler)

# gracefully terminate the script if a terminate signal is received
# (e.g. kill -TERM).
def sigterm_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True
    global exit_code
    exit_code = 0

signal.signal(signal.SIGTERM, sigterm_handler)

send_msg_to_gcs('Sending vision messages to FCU')

try:
    while not main_loop_should_quit:
        rospy.spin()
except Exception as e: progress(e)
except:
    send_msg_to_gcs('ERROR IN SCRIPT')
    progress("Unexpected error: %s" % sys.exc_info()[0])
finally:
    progress('Closing the script...')
    # start a timer in case stopping everything nicely doesn't work.
    signal.setitimer(signal.ITIMER_REAL, 5)  # seconds...
    mavlink_thread_should_exit = True
    mavlink_thread.join()
    conn.close()
    sys.exit(exit_code)
