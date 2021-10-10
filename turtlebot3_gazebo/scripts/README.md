```c++
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch x_pos:="-5" y_pos:="5.5"
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

//1.
rosrun record follow_wall.py
//2.
rosrun record follow_wall_PID.py

// draw imu
rosrun record record_node

// draw the result
rosrun record draw_data.py
```
