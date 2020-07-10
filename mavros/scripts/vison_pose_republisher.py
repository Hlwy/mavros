#!/usr/bin/env python
import rospy, tf
import os, csv, time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

class mavros_vision_pose_republisher:
    def __init__(self):
        rospy.init_node('mavros_vision_pose_republisher')
        self.ns = rospy.get_namespace()
        update_rate = rospy.get_param('~update_rate', 10)

        output_topic = rospy.get_param('~output_topic', "mavros/vision_pose/pose")
        input_topic = rospy.get_param('~input_topic', "/vo")

        self.in_subber = rospy.Subscriber(input_topic, Odometry, self.odomCallback)
        self.out_pub = rospy.Publisher(output_topic, PoseStamped, queue_size=10)
        # self.out_pub = rospy.Publisher(output_topic, PoseWithCovarianceStamped, queue_size=10)
        self.r = rospy.Rate(update_rate)
        print("[INFO] mavros_vision_pose_republisher Node --- Started...")

    def odomCallback(self, msg):
        outMsg = PoseStamped()
        # outMsg = PoseWithCovarianceStamped()
        outMsg.header = msg.header
        outMsg.pose = msg.pose.pose
        # outMsg.pose.pose = msg.pose.pose
        # outMsg.pose.covariance = msg.pose.covariance
        self.out_pub.publish(outMsg)

    def start(self, debug=False):
        while not rospy.is_shutdown():
            self.r.sleep()

if __name__ == "__main__" :
    mNode = mavros_vision_pose_republisher()
    mNode.start()
