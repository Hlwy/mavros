#!/usr/bin/env python
import math
import threading
import rospy, tf
import numpy as np
import os, csv, time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, quaternion_matrix, quaternion_from_matrix
from std_srvs.srv import Empty
from rtabmap_ros.srv import ResetPose

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

    try:
        _listener.waitForTransform(base_frame,child_frame, rospy.Time(0), rospy.Duration(8.0))
        (trans,rot) = _listener.lookupTransform(base_frame,child_frame, rospy.Time(0))
        offset_to_world = np.matrix(tf.transformations.quaternion_matrix(rot))
        offset_to_world[0,3] = trans[0]
        offset_to_world[1,3] = trans[1]
        offset_to_world[2,3] = trans[2]
        return offset_to_world
    except:
        rospy.logwarn("mavros_vision_pose_republisher Node --- Unable to get tf information for transformation between \'%s\' to \'%s\'." % (base_frame, child_frame) )
        return None
def callRosService(srv_name, srv_data = None, verbose = False):
    execSrv = None; resp = None;
    if(verbose): print("Waiting for Service \'%s\'..." % (srv_name))
    rospy.wait_for_service(srv_name)
    try:
        if(verbose): print("Executing Service \'%s\'..." % (srv_name))
        if(srv_data is not None):
            execSrv = rospy.ServiceProxy(srv_name, ResetPose)
            # resp = execSrv(srv_data)
            # resp = execSrv(srv_data[0], srv_data[1], srv_data[2], srv_data[3], srv_data[4], srv_data[5])
            resp = execSrv(srv_data[0], srv_data[1], srv_data[2], srv_data[3], srv_data[4], -1.5717)
        else:
            execSrv = rospy.ServiceProxy(srv_name, Empty)
            resp = execSrv()

        if(verbose): print("Service call to \'%s\' returned successfully"% (srv_name))
        return True
        # if(srv_data is None):
        #     print("Service call to \'%s\' returned successfully"% (srv_name))
        #     return True
        # else: print("[ERROR] Service call to \'%s\' failed." % (srv_name))
    except rospy.ServiceException, e: print("Service call to \'%s\' failed: %s"% (srv_name,e))
    return False

class mavros_vision_pose_republisher:
    lock = threading.Lock()
    RotMat = None
    poseMsg = None
    pose_offset = None
    quat_offset = None

    H_aeroRef_T265Ref = None
    H_T265body_aeroBody = None

    H_aeroRef_aeroBody = None
    H_aeroRef_PrevAeroBody = None

    allow_msg_pub = False
    use_vo_estimate = True
    recvd_initial_pose = False
    rtabResetId = 'rtabmap/reset_odom'
    rtabResetPoseId = 'rtabmap/reset_odom_to_pose'
    def __init__(self):
        camera_orientation = 0
        if camera_orientation == 0:     # Forward, USB port to the right
            self.H_aeroRef_T265Ref   = np.array([
                [0, 0, -1, 0],
                [1, 0,  0, 0],
                [0,-1,  0, 0],
                [0, 0,  0, 1]
            ])
            self.H_T265body_aeroBody = np.linalg.inv(self.H_aeroRef_T265Ref)
        elif camera_orientation == 1:   # Downfacing, USB port to the right
            self.H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
            self.H_T265body_aeroBody = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
        elif camera_orientation == 2:   # 45degree forward
            self.H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
            self.H_T265body_aeroBody = (euler_matrix(math.pi/4, 0, 0)).dot(np.linalg.inv(self.H_aeroRef_T265Ref))
        else:                           # Default is facing forward, USB port to the right
            self.H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
            self.H_T265body_aeroBody = np.linalg.inv(self.H_aeroRef_T265Ref)

        rospy.init_node('mavros_vision_pose_republisher')
        self.ns = rospy.get_namespace()

        self.tf_frame = rospy.get_param('~output_tf', "odom")
        update_rate = rospy.get_param('~update_rate', 30)
        self.flag_use_pose_offsets = rospy.get_param('~use_pose_offsets', False)

        vo_topic = rospy.get_param('~vo_topic', "/vo")
        lsm_topic = rospy.get_param('~lsm_topic', "lsm/corrected_pose")
        pose_topic = rospy.get_param('~pose_topic', "mavros/local_position/pose")
        output_topic = rospy.get_param('~output_topic', "mavros/vision_pose/pose_cov")
        speed_topic = rospy.get_param('~speed_topic', "mavros/vision_speed/speed_twist_cov")
        rospy.Subscriber(vo_topic, Odometry, self.voCallback)
        rospy.Subscriber(lsm_topic, PoseStamped, self.lsmCallback)
        rospy.Subscriber(pose_topic, PoseStamped, self.poseCallback)

        self.out_pub = rospy.Publisher(output_topic, PoseWithCovarianceStamped, queue_size=10)
        self.speed_pub = rospy.Publisher(speed_topic, TwistWithCovarianceStamped, queue_size=10)

        self.reset_srv = rospy.Service("vision_pose_republisher/reset", Empty, self.reset_vo)
        self.vo_input_srv = rospy.Service("vision_pose_republisher/use_vo", Empty, self.switch_vo)
        self.lsm_input_srv = rospy.Service("vision_pose_republisher/use_lsm", Empty, self.switch_lsm)
        self.r = rospy.Rate(update_rate)

        if(self.use_vo_estimate): callRosService(self.rtabResetId, None)

        tfMavFrdAltChild = "fcu"
        tfMavFrdAltBase  = "fcu_frd"
        tfMavFrdChild    = "base_link"
        tfMavFrdBase     = "base_link_frd"
        self.listener_ = tf.TransformListener()
        rotationMatrix = get_tf_frame_pose(child_frame=tfMavFrdChild, base_frame=tfMavFrdBase, listener=self.listener_)
        if(rotationMatrix is None):
            rospy.logwarn("mavros_vision_pose_republisher Node --- Attempting to use alternative tryef frames \'%s\' to \'%s\'." % (tfMavFrdAltChild, tfMavFrdAltBase) )
            rotationMatrix = get_tf_frame_pose(child_frame=tfMavFrdAltChild, base_frame=tfMavFrdAltBase, listener=self.listener_)

        self.RotMat = rotationMatrix
        rospy.loginfo("mavros_vision_pose_republisher Node --- Started...")

    def reset_vo(self, req):
        rospy.loginfo("mavros_vision_pose_republisher --- Resetting...")
        self.recvd_initial_pose = False
        self.allow_msg_pub = False
        self.pose_offset = None
        if(self.use_vo_estimate): callRosService(self.rtabResetId, None)
        return []
    def switch_vo(self, req):
        rospy.loginfo("mavros_vision_pose_republisher --- Switching Mavros EKF Ext. Nav Source to VO.")
        self.use_vo_estimate = True
        return []
    def switch_lsm(self, req):
        rospy.loginfo("mavros_vision_pose_republisher --- Switching Mavros EKF Ext. Nav Source to LSM.")
        self.use_vo_estimate = False
        return []

    def poseCallback(self, msg):
        if(not self.recvd_initial_pose):
            data =  msg.pose
            (x,y,z,r,p,yaw) = quat2euler(data)
            self.pose_offset = [x,y,z,r,p,yaw]
            self.recvd_initial_pose = True
            print("[INFO] mavros_vision_pose_republisher Node --- Recieved Initial Pose = %.4f, %.4f, %.4f - At - %.4f, %.4f, %.4f"
                % (x,y,z,np.rad2deg(r), np.rad2deg(p), np.rad2deg(yaw))
            )
            self.allow_msg_pub = True

    def voCallback(self, msg):
        poseIn = msg.pose.pose
        twistIn = msg.twist
        if(self.use_vo_estimate):
            outMsg = PoseWithCovarianceStamped()
            spdMsg = TwistWithCovarianceStamped()
            outMsg.header = msg.header
            outMsg.header.frame_id = self.tf_frame
            outMsg.pose.pose = poseIn
            spdMsg.header = msg.header
            spdMsg.header.frame_id = self.tf_frame
            spdMsg.twist = twistIn

            """ Extracted from vision_to_mavros repo """
            # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
            H_T265Ref_T265body = quaternion_matrix([ msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])
            H_T265Ref_T265body[0][3] = msg.pose.pose.position.x
            H_T265Ref_T265body[1][3] = msg.pose.pose.position.y
            H_T265Ref_T265body[2][3] = msg.pose.pose.position.z
            # Transform to aeronautic coordinates (body AND reference frame!)
            HrefToBody = self.H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( self.H_T265body_aeroBody))

            # Start Camera Offsets
            H_body_camera = euler_matrix(0, 1.5717, -1.5717, 'sxyz')
            H_body_camera[0][3] = 0.0
            H_body_camera[1][3] = 0.0
            H_body_camera[2][3] = 0.0
            H_camera_body = np.linalg.inv(H_body_camera)
            HrefToBody = H_body_camera.dot(HrefToBody.dot(H_camera_body))
            # END Camera Offsets

            self.H_aeroRef_aeroBody = HrefToBody

            # TEST SECTION
            delta_angle_rad  = np.array( quaternion_from_matrix(HrefToBody) )
            outMsg.pose.pose.position.x = HrefToBody[0][3]
            outMsg.pose.pose.position.y = HrefToBody[1][3]
            outMsg.pose.pose.orientation.x = delta_angle_rad[1]
            outMsg.pose.pose.orientation.y = delta_angle_rad[2]
            outMsg.pose.pose.orientation.z = delta_angle_rad[3]
            outMsg.pose.pose.orientation.w = delta_angle_rad[0]

            outMsg.pose.covariance = msg.pose.covariance
            self.out_pub.publish(outMsg)
            self.speed_pub.publish(spdMsg)

    def lsmCallback(self, msg):
        poseIn = msg.pose
        if(not self.use_vo_estimate):
            outMsg = PoseWithCovarianceStamped()
            outMsg.header = msg.header
            outMsg.header.frame_id = self.tf_frame
            outMsg.pose.pose = poseIn
            self.out_pub.publish(outMsg)


    def start(self, debug=False):
        while not rospy.is_shutdown():
            self.r.sleep()

if __name__ == "__main__" :
    mNode = mavros_vision_pose_republisher()
    mNode.start()
