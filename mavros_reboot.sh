#!/bin/bash

# Get absolute path of this script to get the catkin ws root path for sourcing
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
CATKIN_DEVEL_PATH=$(echo $SCRIPTPATH | sed -e 's;\/[^/]*$;;' | sed -e 's;\/[^/]*$;;' | sed -e 's;\/[^/]*$;;')
CATKIN_SOURCE_PATH="$CATKIN_DEVEL_PATH/devel/setup.bash"

# Check if a ROS namespace has been set to run necessary commands
# for starting ROS nodes w/ appropriate namespacing
ROS_NS=""
MAVROS_NS=""
if [ -z ${ROS_NAMESPACE+x} ]; then
     ROS_NS=""
     MAVROS_NS="mavros"
else
     ROS_NS="/${ROS_NAMESPACE}"
     MAVROS_NS="/${ROS_NAMESPACE}/mavros"
fi

# Disarm autopilot so that the autopilot will accept our request for reboot
echo " ---------------------------------------- "
echo "           Disarming Autopilot...         "
echo " ---------------------------------------- "
sleep 1
(source $CATKIN_SOURCE_PATH; rosrun mavros mavsafety -n "${MAVROS_NS}" disarm)

# Send the request for autopilot reboot to the autopilot
echo " ---------------------------------------- "
echo "           Rebooting Autopilot...         "
echo " ---------------------------------------- "
sleep 1

REBOOT_CMD=$(cat <<- END
{broadcast: false, command: 246, confirmation: 0, param1: 1,
  param2: 0.0, param3: 0.0,
  param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}
END
)
rosservice call /$MAVROS_NS/cmd/command "$REBOOT_CMD"
