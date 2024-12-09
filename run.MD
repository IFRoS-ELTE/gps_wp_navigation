
case 1 :
```sh
  1. roslaunch scout_bringup scout_minimal.launch
  2. roslaunch scout_bringup sensors.launch
  3. rosrun scout_robot__2dnav imu_vis.py
  4. rosrun scout_robot__2dnav gps_navigation_node.py
  5. roslaunch scout_robot__2dnav move_base_mapless.launch
  ```

case 2 : RRT
```sh
  1. roslaunch scout_bringup scout_minimal.launch
  2. roslaunch scout_bringup sensors.launch
  3. rosrun scout_robot__2dnav imu_vis.py
  4. rosrun scout_robot__2dnav gps_navigation_node.py
  5. roslaunch scout_robot__2dnav scout_planner.launch

  ```

  case 3 : EKF localization
```sh
  1. roslaunch scout_bringup scout_minimal.launch # from catkin_ws
  2. roslaunch scout_bringup sensors.launch
  3. rosrun scout_robot__2dnav imu_vis.py
  4. rosrun scout_robot__2dnav gps_navigation_node.py
  5. roslaunch scout_robot__2dnav scout_planner.launch
  
  6. roslaunch scout_robot__2dnav ekf_localization.launch
  7. roslaunch scout_robot__2dnav navsattransform.launch

  ```