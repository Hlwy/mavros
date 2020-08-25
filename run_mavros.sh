#!/bin/bash
trap "trap - SIGTERM && kill -- $$" SIGINT SIGTERM EXIT
unset ROS_IP
unset ROS_NAMESPACE
unset ROS_MASTER_URI
wlanIp="$(ip addr show wlan0 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p' | head -1)"
#ROBOT_IP="$(ifconfig | grep -A 1 'wl' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)"
#ROBOT_IP="$(ifconfig $dfltIface | sed -En -e 's/.*inet addr: ([0-9.]+).*/\1/p')"
#rosIp="$(ip addr show $rosIface | awk '$1 == "inet" {gsub(/\/.*$/, "", $2); print $2}')"
# rosIp="$(ip addr show wlan0 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p' | head -1)"
rosIp="$(ip addr show eth0 | sed -En -e 's/.*inet ([0-9.]+).*/\1/p' | head -1)"
if [[ -z "$rosIp" ]]; then
     echo "   ROS_IP         = Unsetting"
elif [[ -n "$rosIp" ]]; then
     echo "   ROS_IP         = $rosIp"
     export ROS_IP=$rosIp
fi

# rosMasterAddr="localhost"
# rosMasterAddr=10.0.0.133
rosMasterAddr=192.168.2.3
rosMaster="http://$rosMasterAddr:11311"
if [[ -z "$rosMaster" ]]; then
     echo "   ROS_MASTER_URI = Unsetting"
elif [[ -n "$rosMaster" ]]; then
     echo "   ROS_MASTER_URI = $rosMaster"
fi
export ROS_MASTER_URI=$rosMaster

# (source /swanson/catkin_ws/devel/setup.bash; rosservice call /rtabmap/pause_odom "{}")&
(source /swanson/catkin_ws/devel/setup.bash; roslaunch mavros apm_custom.launch gcs_url:="udp://$wlanIp:9000@10.0.0.74:6000?ids=1,255,252")&
# (source /swanson/catkin_ws/devel/setup.bash; roslaunch mavros apm_custom.launch gcs_url:="udp://$rosIp:9000@10.0.0.133:6000?ids=1,255,252")&
# (source /swanson/catkin_ws/devel/setup.bash; roslaunch mavros apm_custom.launch gcs_url:="udp://$rosIp:9000@192.168.2.4:6000?ids=1,255,252")&
# (source /swanson/catkin_ws/devel/setup.bash; roslaunch mavros apm_custom.launch gcs_url:="udp://$rosIp:9000@10.0.0.70:6000?ids=1,255,252")&
(source /swanson/catkin_ws/devel/setup.bash; roslaunch mavros initialize_ekf.launch)&
sleep 10
(source /swanson/catkin_ws/devel/setup.bash; rosrun mavros mavsys mode -c GUIDED)& # APM
# (source /swanson/catkin_ws/devel/setup.bash; rosrun mavros mavsys mode -c 15)& # PX4
# (source /swanson/catkin_ws/devel/setup.bash; rosrun mavros mavsafety arm)&
# (source /swanson/catkin_ws/devel/setup.bash; rosrun mavros altitude_fixer.py)&
sleep 5
# (source /swanson/catkin_ws/devel/setup.bash; roslaunch swanson_sensors camera_d4xx.launch)&
# (source /swanson/catkin_ws/devel/setup.bash; rosservice call /rtabmap/reset_odom "{}")&
# sleep 5
# (source /swanson/catkin_ws/devel/setup.bash; rosrun mavros vison_pose_republisher.py)&
(source /swanson/catkin_ws/devel/setup.bash; rosrun mavros mavsys rate --position 20)&
(source /swanson/catkin_ws/devel/setup.bash; rosrun mavros mavsys rate --raw-sensors 10)&
read asdf
