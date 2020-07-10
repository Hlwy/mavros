#!/usr/bin/env python
import rospy
import argparse
from mavros_msgs.msg import *
from mavros_msgs.srv import *

def setArm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(True)
   except rospy.ServiceException, e: print("Service arm call failed: %s"%e)

def setDisarm():
   rospy.wait_for_service('/mavros/cmd/arming')
   try:
       armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
       armService(False)
   except rospy.ServiceException, e: print("Service arm call failed: %s"%e)

def setGuidedMode():
   rospy.wait_for_service('/mavros/set_mode')
   try:
       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
   except rospy.ServiceException, e:
       print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled")
def setManualMode():
   rospy.wait_for_service('/mavros/set_mode')
   try:
       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       isModeChanged = flightModeService(custom_mode='MANUAL') #return true or false
   except rospy.ServiceException, e:
       print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled")
def setHoldMode():
   rospy.wait_for_service('/mavros/set_mode')
   try:
       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       isModeChanged = flightModeService(custom_mode='HOLD') #return true or false
   except rospy.ServiceException, e:
       print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled")


if __name__ == '__main__':
    # Setup commandline argument(s) structures
    ap = argparse.ArgumentParser(description='')
    ap.add_argument("--mode", "-m", type=str, default='hold', metavar='MODE', help="")
    ap.add_argument("--arm", "-a", action="store_true", help="")
    args = vars(ap.parse_args()) # Store parsed arguments into array of variables

    # Extract stored arguments array into individual variables for later usage in script
    mavMode = args["mode"]
    flagArm = args["arm"]

    if(flagArm): setArm()
    else: setDisarm()

    if(mavMode == "guided"):
        print("Setting MAV Mode to Guided...")
        setGuidedMode()
    elif(mavMode == "manual"):
        print("Setting MAV Mode to Manual...")
        setManualMode()
    elif(mavMode == "hold"):
        print("Setting MAV Mode to Hold...")
        setHoldMode()
    else:
        print("Unknown MAV mode given, Defaulting to Hold mode...")
        setHoldMode()
