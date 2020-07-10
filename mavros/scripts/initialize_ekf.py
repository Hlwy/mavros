#!/usr/bin/env python
import rospy
import sys, time
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix

class MavrosEkfInitializer(object):
    _max_retry_attempts = 4
    _recvd_local_pose = False
    _cur_global_pos = NavSatFix()

    def __init__(self, name):
        rospy.init_node(name, anonymous=False)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.localPoseCallback)
        rospy.Subscriber('mavros/global_position/global', NavSatFix, self.globalPoseCallback)
        self._origin_pubber = rospy.Publisher('mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)

        # Pass these arguments in to be used in case global position is not available
        self._latitude = rospy.get_param('~fallback_latitude')
        self._longitude = rospy.get_param('~fallback_longitude')

        time.sleep(5.0)
        while(not self._recvd_local_pose and self._max_retry_attempts >= 0):
            self.set_ekf_origin()
            time.sleep(3.0)
            self._max_retry_attempts -= 1
        # Check to see if EKF initialized successfully
        if(self._recvd_local_pose): rospy.loginfo("MavrosEkfInitializer() --- EKF has been successfully initialized!")
        else: rospy.logerr("MavrosEkfInitializer() --- Unable to successfully initialize EKF.")


    def globalPoseCallback(self, msg):
        self._cur_global_pos = msg

    def localPoseCallback(self, msg):
        self._recvd_local_pose = True

    def set_ekf_origin(self):
        msg = GeoPointStamped()
        msg.position.altitude = 0.0 # We don't care about altitude
        if(self._cur_global_pos.latitude == 0.0):
            msg.position.latitude = self._latitude
            msg.position.longitude = self._longitude
        else:
            msg.position.latitude = self._cur_global_pos.latitude
            msg.position.longitude = self._cur_global_pos.longitude
        msg.position.altitude = 0.0
        self._origin_pubber.publish(msg)

if __name__ == '__main__':
    MavrosEkfInitializer('mavros_ekf_initializer')
