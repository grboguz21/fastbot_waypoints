TASK 2 
------------------

# PASSING CONDITION
In order to test the passing condition, navigate to: 
/home/user/ros2_ws/src/fastbot_waypoints/test/control_node_test.cpp

replace the lines (95-97) with the following code:
  Waypoint::Goal goal;
  goal.position.x = 2.0;
  goal.position.y = 2.0;


# FAILLING CONDITION 
In order to test failing condition, navigate to: 
/home/user/ros2_ws/src/fastbot_waypoints/test/control_node_test.cpp

replace the lines (95-97) with the following code:
  Waypoint::Goal goal;
  goal.position.x = 4.0;
  goal.position.y = 2.0;