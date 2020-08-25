#!/bin/bash
rosrun mavros mavsafety disarm

CMD1=$(cat <<- END
{broadcast: false, command: 246, confirmation: 0, param1: 1,
  param2: 0.0, param3: 0.0,
  param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}
END
)

rosservice call /mavros/cmd/command "$CMD1"
