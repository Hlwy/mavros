#!/usr/bin/env python

import os, sys
import rospy, tf
import numpy as np
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

def get_tf_frame_pose(child_frame = 'base_link', base_frame = 'odom', listener=None):
    if(listener is None): _listener = tf.TransformListener()
    else: _listener = listener

    _listener.waitForTransform(base_frame,child_frame, rospy.Time(0), rospy.Duration(8.0))
    (trans,rot) = _listener.lookupTransform(base_frame,child_frame, rospy.Time(0))
    roll,pitch,yaw = tf.transformations.euler_from_quaternion(rot)

    pose = np.array(trans+[np.rad2deg(yaw)])
    Tmat = np.array(trans).T
    Rmat = tf.transformations.euler_matrix(roll,pitch,yaw,axes='sxyz')
    return pose, Rmat, Tmat, rot, [roll,pitch,yaw]

class altitude_corrector:
    def __init__(self):
        rospy.init_node('altitude_corrector')

        self.ns = rospy.get_namespace()
        update_rate = rospy.get_param('~update_rate', 50)

        self.target_height = rospy.get_param('~target_height', 0.2)
        self.base_tf_frame = rospy.get_param('~base_tf_frame', "world")
        self.child_tf_frame = rospy.get_param('~child_tf_frame', "map")
        self.altitude_topic = rospy.get_param('~altitude_topic', "mavros/global_position/rel_alt")
        self.alt_subber = rospy.Subscriber(self.altitude_topic,Float64, self.altitudeCallback)
        self.br = tf.TransformBroadcaster()
        self.tfListener = tf.TransformListener()

        self.r = rospy.Rate(update_rate)
        self.count = 0
        print("[INFO] altitude_corrector Node --- Started...")

    def altitudeCallback(self, msg):
        curT = rospy.Time.now()
        dAlt = -(msg.data - self.target_height)
        # rospy.logdebug("altitude_corrector() --- Publishing correcting tf from child frame \'%s\' -> parent frame \'%s\'. Altitude Correction = cur_alt - target_alt = %.3f = %.3f - %.3f." % (self.child_tf_frame, self.base_tf_frame, dAlt, msg.data, self.target_height) )
        # self.br.sendTransform((0,0,dAlt), tf.transformations.quaternion_from_euler(0, 0, 0), curT, self.child_tf_frame,self.base_tf_frame)

    def start(self):
        robot_frame_id = "base_link"
        mavros_odom_frame_id = "map"
        while not rospy.is_shutdown():
            pos, _, _, rot, _ = get_tf_frame_pose(child_frame = robot_frame_id, base_frame=mavros_odom_frame_id, listener=self.tfListener)
            curAlt = float(pos[2])
            dAlt = self.target_height - curAlt
            rospy.loginfo("altitude_corrector() --- Current altitude between tf's \'%s\' -> \'%s\' = %.3f." % (robot_frame_id, mavros_odom_frame_id, curAlt) )
            self.br.sendTransform((0,0, -dAlt), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), self.child_tf_frame,self.base_tf_frame)
            self.count+=1
            self.r.sleep()

if __name__ == "__main__" :
    mNode = altitude_corrector()
    mNode.start()
