#!/usr/bin/env python
import rospy, tf
import numpy as np
import os, time, math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def quat2euler(data):
    quats = (data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w)
    euler = euler_from_quaternion(quats)

    x = data.position.x
    y = data.position.y
    z = data.position.z
    r = euler[0]
    p = euler[1]
    yaw = euler[2]

    pose = [x,y,z,r,p,yaw]
    return pose
def get_tf_frame_pose(child_frame = 'base_link',base_frame = 'base_link_frd', listener=None):
    if(listener is None): _listener = tf.TransformListener()
    else: _listener = listener
    _listener.waitForTransform(base_frame,child_frame, rospy.Time(0), rospy.Duration(8.0))
    (trans,rot) = _listener.lookupTransform(base_frame,child_frame, rospy.Time(0))
    offset_to_world = np.matrix(tf.transformations.quaternion_matrix(rot))
    offset_to_world[0,3] = trans[0]
    offset_to_world[1,3] = trans[1]
    offset_to_world[2,3] = trans[2]
    return offset_to_world

class mavros_encoder_pose_republisher:
    pose_offset = None
    recvd_initial_pose = False
    def __init__(self):
        rospy.init_node('mavros_encoder_pose_republisher')
        self.ns = rospy.get_namespace()
        self.tf_frame = rospy.get_param('~output_tf', "odom")
        self.flag_use_pose_offsets = rospy.get_param('~use_pose_offsets', False)

        update_rate = rospy.get_param('~update_rate', 30)
        input_topic = rospy.get_param('~input_topic', "mavros/wheel_encoders_data/odom")
        output_topic = rospy.get_param('~output_topic', "mavros/encoder_pose/pose_estimate")

        self.in_subber = rospy.Subscriber(input_topic, Odometry, self.estimateCallback)
        self.out_pub = rospy.Publisher(output_topic, Odometry, queue_size=10)

        self.r = rospy.Rate(update_rate)
        print("[INFO] mavros_encoder_pose_republisher Node --- Started...")

    def estimateCallback(self, msg):
        data =  msg.pose.pose
        (x,y,z,r,p,yaw) = quat2euler(data)

        outMsg = Odometry()
        if(self.pose_offset is not None):
            quats = quaternion_from_euler(
                r - self.pose_offset[3],
                p - self.pose_offset[4],
                (yaw-np.deg2rad(90.0)) - self.pose_offset[5]
            )
            outMsg.header = msg.header
            outMsg.pose = msg.pose
            outMsg.pose.pose.position.x = y - self.pose_offset[1]
            outMsg.pose.pose.position.y = -(x - self.pose_offset[0])
            outMsg.pose.pose.orientation.x = quats[0]
            outMsg.pose.pose.orientation.y = quats[1]
            outMsg.pose.pose.orientation.z = quats[2]
            outMsg.pose.pose.orientation.w = quats[3]
            self.out_pub.publish(outMsg)
        else: self.pose_offset = [x,y,z,r,p,yaw-np.deg2rad(90.0)]

    def poseCallback(self, msg):
        if(self.pose_offset is None):
            data =  msg.pose
            (x,y,z,r,p,yaw) = quat2euler(data)
            self.pose_offset = [x,y,z,r,p,yaw]
            self.recvd_initial_pose = True
            print("[INFO] mavros_encoder_pose_republisher Node --- Recieved Initial Pose = %.4f, %.4f, %.4f - At - %.4f, %.4f, %.4f"
                % (x,y,z,np.rad2deg(r), np.rad2deg(p), np.rad2deg(yaw))
            )

    def start(self, debug=False):
        while not rospy.is_shutdown():
            self.r.sleep()

if __name__ == "__main__" :
    mNode = mavros_encoder_pose_republisher()
    mNode.start()
