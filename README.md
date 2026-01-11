error_pos: 0.051->0.06
error_yaw: 0.025->0.03

PASSING CONDITION
-------------------
user:~/ros2_ws$ ros2 action send_goal /fastbot_as fastbot_waypoints/action/Waypoint "{position: {x: 1.4, y: 1.0, z: 0.0}}"
Waiting for an action server to become available...
Sending goal:
     position:
  x: 1.4
  y: 1.0
  z: 0.0

Goal accepted with ID: c45bae006a7d4d32be20e170a718ee72

Result:
    success: true

Goal finished with status: SUCCEEDED
user:~/ros2_ws$ ros2 action send_goal /fastbot_as fastbot_waypoints/action/Waypoint "{position: {x: 1.4, y: 1.0, z: 0.0}}"
Waiting for an action server to become available...
Sending goal:
     position:
  x: 1.4
  y: 1.1
  z: 0.0

Goal accepted with ID: c45bae006a7d4d32be20e170a718ee72

Result:
    success: true

Goal finished with status: SUCCEEDED







FAILLING CONDITION
-------------------
user:~/ros2_ws$ ros2 action send_goal /fastbot_as 
fastbot_waypoints/action/Waypoint "{position: {x: 2.5, y: 2.2, z: 0.0}}"

