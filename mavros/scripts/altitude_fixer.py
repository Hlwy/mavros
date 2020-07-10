#!/usr/bin/env python

import os, sys
import rospy, tf
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class altitude_corrector:
    def __init__(self):
        rospy.init_node('altitude_corrector')

        self.ns = rospy.get_namespace()
        update_rate = rospy.get_param('~update_rate', 100)

        self.target_height = rospy.get_param('~target_height', 0.0)
        self.base_tf_frame = rospy.get_param('~base_tf_frame', "map")
        self.child_tf_frame = rospy.get_param('~child_tf_frame', "odom")
        self.altitude_topic = rospy.get_param('~altitude_topic', "mavros/global_position/rel_alt")
        self.alt_subber = rospy.Subscriber(self.altitude_topic,Float64, self.altitudeCallback)
        self.br = tf.TransformBroadcaster()
        self.r = rospy.Rate(update_rate)
        self.count = 0
        print("[INFO] altitude_corrector Node --- Started...")

    def altitudeCallback(self, msg):
        curT = rospy.Time.now()
        dAlt = msg.data - self.target_height
        self.br.sendTransform((0,0,-dAlt), tf.transformations.quaternion_from_euler(0, 0, 0), curT, self.child_tf_frame,self.base_tf_frame)

    def start(self):
        while not rospy.is_shutdown():
            self.count+=1
            self.r.sleep()

if __name__ == "__main__" :
    mNode = altitude_corrector()
    mNode.start()
