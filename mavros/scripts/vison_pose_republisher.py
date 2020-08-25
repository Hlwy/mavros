#!/usr/bin/env python
import threading
import rospy, tf
import numpy as np
import os, csv, time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty
from rtabmap_ros.srv import ResetPose

import math
import transformations as tfs

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
    RotMat = None
    poseMsg = None
    pose_offset = None
    quat_offset = None

    H_aeroRef_T265Ref = None
    H_T265body_aeroBody = None

    H_aeroRef_aeroBody = None
    H_aeroRef_PrevAeroBody = None

    allow_msg_pub = False
    recvd_initial_pose = False
    rtabPauseId = 'rtabmap/pause_odom'
    rtabPlayId = 'rtabmap/resume_odom'
    rtabResetId = 'rtabmap/reset_odom'
    rtabResetPoseId = 'rtabmap/reset_odom_to_pose'
    lock = threading.Lock()
    def __init__(self):
        rospy.init_node('mavros_vision_pose_republisher')
        self.ns = rospy.get_namespace()

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
            self.H_T265body_aeroBody = (tfs.euler_matrix(math.pi/4, 0, 0)).dot(np.linalg.inv(self.H_aeroRef_T265Ref))
        else:                           # Default is facing forward, USB port to the right
            self.H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
            self.H_T265body_aeroBody = np.linalg.inv(self.H_aeroRef_T265Ref)


        self.tf_frame = rospy.get_param('~output_tf', "odom")
        update_rate = rospy.get_param('~update_rate', 30)

        output_topic = ""
        input_topic = rospy.get_param('~input_topic', "/vo")
        self.pose_output_type = rospy.get_param('~output_type', 1)
        self.flag_use_pose_offsets = rospy.get_param('~use_pose_offsets', False)
        default_topic_out = rospy.get_param('~output_topic', "/vo_pose")
        pose_topic = rospy.get_param('~pose_topic', "mavros/local_position/pose")
        # default_topic_out = rospy.get_param('~output_topic', "/mavros/vision_pose/pose")
        if(self.pose_output_type == 1): output_topic = "/mavros/vision_pose/pose_cov"
        elif(self.pose_output_type == 2):
            self.allow_msg_pub = True
            output_topic = "/mavros/fake_gps/vision"
        else: output_topic = default_topic_out

        sec_output_topic = default_topic_out
        self.sec_out_pub = rospy.Publisher(sec_output_topic, PoseStamped, queue_size=10)

        self.out_pub = None
        self.pose_subber = rospy.Subscriber(pose_topic, PoseStamped, self.poseCallback)
        self.in_subber = rospy.Subscriber(input_topic, Odometry, self.odomCallback)

        if(self.pose_output_type == 1): self.out_pub = rospy.Publisher(output_topic, PoseWithCovarianceStamped, queue_size=10)
        else: self.out_pub = rospy.Publisher(output_topic, PoseStamped, queue_size=10)

        self.r = rospy.Rate(update_rate)
        print("[INFO] mavros_vision_pose_republisher Node --- Started...")

        # callRosService(self.rtabPauseId, None)
        if(self.pose_output_type == 2): callRosService(self.rtabResetPoseId, [0.0, 0.0, 0.0, 0.0, 0.0, np.deg2rad(90.0)])
        else: callRosService(self.rtabResetId, None)
        self.RotMat = get_tf_frame_pose()

    def poseCallback(self, msg):
        if(not self.recvd_initial_pose and self.pose_output_type != 2):
            data =  msg.pose
            (x,y,z,r,p,yaw) = quat2euler(data)
            self.pose_offset = [x,y,z,r,p,yaw]
            # self.quat_offset = [data.orientation.x, data.orientation.y,data.orientation.z,data.orientation.w]
            # self.quat_offset = [r,p,yaw]
            self.recvd_initial_pose = True
            print("[INFO] mavros_vision_pose_republisher Node --- Recieved Initial Pose = %.4f, %.4f, %.4f - At - %.4f, %.4f, %.4f"
                % (x,y,z,np.rad2deg(r), np.rad2deg(p), np.rad2deg(yaw))
            )

            # callRosService(self.rtabResetPoseId, self.pose_offset)
            # callRosService(self.rtabResetId, None)
            # callRosService(self.rtabPlayId, None)
            self.allow_msg_pub = True

    def odomCallback(self, msg):
        poseIn = msg.pose.pose
        if(self.pose_output_type == 1):
            outMsg = PoseWithCovarianceStamped()
            outMsg.header = msg.header
            outMsg.header.frame_id = self.tf_frame
            outMsg.pose.pose = poseIn

            """ Extracted from vision_to_mavros repo """
            # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
            H_T265Ref_T265body = tfs.quaternion_matrix([
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ])
            H_T265Ref_T265body[0][3] = msg.pose.pose.position.x
            H_T265Ref_T265body[1][3] = msg.pose.pose.position.y
            H_T265Ref_T265body[2][3] = msg.pose.pose.position.z
            # Transform to aeronautic coordinates (body AND reference frame!)
            HrefToBody = self.H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( self.H_T265body_aeroBody))

            # Start Camera Offsets
            H_body_camera = tfs.euler_matrix(0, 1.5717, -1.5717, 'sxyz')
            H_body_camera[0][3] = 0.0
            H_body_camera[1][3] = 0.0
            H_body_camera[2][3] = 0.0
            H_camera_body = np.linalg.inv(H_body_camera)
            HrefToBody = H_body_camera.dot(HrefToBody.dot(H_camera_body))
            # END Camera Offsets

            self.H_aeroRef_aeroBody = HrefToBody

            """
            # Calculate the deltas in position, attitude and time from the previous to current orientation
            if self.H_aeroRef_PrevAeroBody is None:
                self.H_aeroRef_PrevAeroBody = self.H_aeroRef_aeroBody
                return
            else:
                H_PrevAeroBody_CurrAeroBody = (np.linalg.inv(self.H_aeroRef_PrevAeroBody)).dot(self.H_aeroRef_aeroBody)
                delta_time_us    = current_time_us - send_vision_position_delta_message.prev_time_us
                delta_position_m = [H_PrevAeroBody_CurrAeroBody[0][3], H_PrevAeroBody_CurrAeroBody[1][3], H_PrevAeroBody_CurrAeroBody[2][3]]
                delta_angle_rad  = np.array( tfs.euler_from_matrix(H_PrevAeroBody_CurrAeroBody, 'sxyz'))

            print("DEBUG: Raw RPY[deg]: {}".format( np.array( tf.euler_from_matrix( H_T265Ref_T265body, 'sxyz')) * 180 / m.pi))
            print("DEBUG: NED RPY[deg]: {}".format( np.array( tf.euler_from_matrix( H_aeroRef_aeroBody, 'sxyz')) * 180 / m.pi))
            print("DEBUG: Raw pos xyz : {}".format( np.array( [data.translation.x, data.translation.y, data.translation.z])))
            print("DEBUG: NED pos xyz : {}".format( np.array( tf.translation_from_matrix( H_aeroRef_aeroBody))))
            """

            """ END --- Extracted from vision_to_mavros repo """

            # Original SECTION
            # tmpX = outMsg.pose.pose.position.x
            # tmpY = outMsg.pose.pose.position.y
            # outMsg.pose.pose.position.x = tmpX
            # outMsg.pose.pose.position.y = tmpY
            # END Original SECTION

            # TEST SECTION
            delta_angle_rad  = np.array( tfs.quaternion_from_matrix(HrefToBody) )
            outMsg.pose.pose.position.x = HrefToBody[0][3]
            outMsg.pose.pose.position.y = HrefToBody[1][3]
            outMsg.pose.pose.orientation.x = delta_angle_rad[1]
            outMsg.pose.pose.orientation.y = delta_angle_rad[2]
            outMsg.pose.pose.orientation.z = delta_angle_rad[3]
            outMsg.pose.pose.orientation.w = delta_angle_rad[0]
            # print("DEBUG: Raw pos xy : {}".format( np.array( [msg.pose.pose.position.x, msg.pose.pose.position.y])))
            # print("DEBUG: NED pos xy : {}".format( np.array( [outMsg.pose.pose.position.x, outMsg.pose.pose.position.y])))
            # print(" --------- ")
            # END TEST SECTION

            outMsg.pose.covariance = msg.pose.covariance
            # print("[INFO] mavros_vision_pose_republisher --- Covariance In = %s" % (str(msg.pose.covariance)))
            # print("[INFO] mavros_vision_pose_republisher --- Covariance Out = %s" % (str(outMsg.pose.covariance)))

            # if(self.recvd_initial_pose and self.flag_use_pose_offsets):
            #     (xIn,yIn,zIn,rIn,pIn,yawIn) = quat2euler(poseIn)
            #     quatsOffset = quaternion_from_euler( (rIn + self.pose_offset[3]), (pIn + self.pose_offset[4]), (yawIn + self.pose_offset[5]) )
            #     outMsg.pose.orientation.x = quatsOffset[0]
            #     outMsg.pose.orientation.y = quatsOffset[1]
            #     outMsg.pose.orientation.z = quatsOffset[2]
            #     outMsg.pose.orientation.w = quatsOffset[3]

            self.out_pub.publish(outMsg)

        else:
            outMsg = PoseStamped()
            outMsg.header = msg.header
            outMsg.header.frame_id = self.tf_frame

            outMsg.pose = poseIn
            tmpX = outMsg.pose.position.x
            tmpY = outMsg.pose.position.y
            outMsg.pose.position.x = tmpX
            outMsg.pose.position.y = tmpY

            if(self.recvd_initial_pose and self.flag_use_pose_offsets):
                (xIn,yIn,zIn,rIn,pIn,yawIn) = quat2euler(poseIn)
                quatsOffset = quaternion_from_euler( (rIn + self.pose_offset[3]), (pIn + self.pose_offset[4]), (yawIn + self.pose_offset[5]) )
                outMsg.pose.orientation.x = quatsOffset[0]
                outMsg.pose.orientation.y = quatsOffset[1]
                outMsg.pose.orientation.z = quatsOffset[2]
                outMsg.pose.orientation.w = quatsOffset[3]

            self.out_pub.publish(outMsg)
            # self.sec_out_pub.publish(outMsg)


    def start(self, debug=False):
        while not rospy.is_shutdown():
            self.r.sleep()

if __name__ == "__main__" :
    mNode = mavros_vision_pose_republisher()
    mNode.start()
