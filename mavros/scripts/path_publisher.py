#!/usr/bin/env python
import rospy, tf
import numpy as np
import os, time, math
from std_srvs.srv import Empty
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def get_euclidean_distance(current_data, prev_data):
    prevX = prev_data.position.x
    prevY = prev_data.position.y
    curX = current_data.position.x
    curY = current_data.position.y

    dx = curX - prevX
    dy = curY - prevY
    dist = math.sqrt(dx*dx + dy*dy)
    return (dist, dx, dy)

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

class mavros_path_publisher:
    debug = False
    verbose = False
    vo_path = Path()
    wenc_path = Path()
    local_path = Path()
    prev_vo_pose = None
    vo_pose_err_thresh = 0.07
    def __init__(self):
        rospy.init_node('mavros_path_publisher')
        self.ns = rospy.get_namespace()
        update_rate = rospy.get_param('~update_rate', 30)

        vo_pose_topic = rospy.get_param('~vo_pose_topic', "vision_pose/pose_estimate")
        wenc_pose_topic = rospy.get_param('~wenc_pose_topic', "mavros/encoder_pose/pose_estimate")
        local_pose_topic = rospy.get_param('~local_pose_topic', "mavros/local_position/pose")

        vo_path_topic = rospy.get_param('~vo_path_topic', "path/vision")
        wenc_path_topic = rospy.get_param('~wenc_path_topic', "path/encoder")
        local_path_topic = rospy.get_param('~local_path_topic', "path/local_ekf")

        self.vo_sub = rospy.Subscriber(vo_pose_topic, PoseWithCovarianceStamped, self.visionCallback)
        self.wenc_sub = rospy.Subscriber(wenc_pose_topic, Odometry, self.encoderCallback)
        self.local_sub = rospy.Subscriber(local_pose_topic, PoseStamped, self.localPoseCallback)

        self.vo_path_pub = rospy.Publisher(vo_path_topic, Path, queue_size=10)
        self.wenc_path_pub = rospy.Publisher(wenc_path_topic, Path, queue_size=10)
        self.local_path_pub = rospy.Publisher(local_path_topic, Path, queue_size=10)

        self.vo_srv = rospy.Service("path_publisher/reset/vo", Empty, self.reset_vo_path)
        self.wenc_srv = rospy.Service("path_publisher/reset/wenc", Empty, self.reset_wenc_path)
        self.ekf_srv = rospy.Service("path_publisher/reset/ekf", Empty, self.reset_ekf_path)
        self.r = rospy.Rate(update_rate)
        print("[INFO] mavros_path_publisher Node --- Started...")

    def visionCallback(self, msg):
        pose_err = 0.00001
        if(self.prev_vo_pose is not None):
            err, dx, dy = get_euclidean_distance(msg.pose.pose, self.prev_vo_pose)
            if(self.debug): print("[INFO] mavros_path_publisher --- Vision Pose Error, dX, dY = %.4f, %.4f, %.4f" % (err, dx, dy) )
            pose_err = err
            if( (err == 0) and (dx == 0) and (dy == 0) ): pose_err = 1.0

        self.vo_path.header = msg.header
        if(pose_err < self.vo_pose_err_thresh):
            if(self.verbose): print("[INFO] mavros_path_publisher ----- Adding Vision Pose to Path. Err = %.4f" % (pose_err) )
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.vo_path.poses.append(pose)

        self.vo_path_pub.publish(self.vo_path)
        self.prev_vo_pose = msg.pose.pose

    def encoderCallback(self, msg):
        self.wenc_path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.wenc_path.poses.append(pose)
        self.wenc_path_pub.publish(self.wenc_path)

    def localPoseCallback(self, msg):
        self.local_path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose
        self.local_path.poses.append(pose)
        self.local_path_pub.publish(self.local_path)

    def reset_vo_path(self, req):
        print("[INFO] mavros_path_publisher --- Resetting Vision Path...")
        path = Path()
        path.header = self.vo_path.header
        self.vo_path = path
        self.vo_path_pub.publish(self.vo_path)
        self.prev_vo_pose = None
        return []
    def reset_wenc_path(self, req):
        print("[INFO] mavros_path_publisher --- Resetting Encoder Path...")
        path = Path()
        path.header = self.wenc_path.header
        self.wenc_path = path
        self.wenc_path_pub.publish(self.wenc_path)
        return []
    def reset_ekf_path(self, req):
        print("[INFO] mavros_path_publisher --- Resetting Estimated Path...")
        path = Path()
        path.header = self.local_path.header
        self.local_path = path
        self.local_path_pub.publish(self.local_path)
        return []

    def start(self, debug=False):
        while not rospy.is_shutdown():
            self.r.sleep()

if __name__ == "__main__" :
    mNode = mavros_path_publisher()
    mNode.start()
