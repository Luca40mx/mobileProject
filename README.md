# Final project for the Mobile Robotics course - A.A. 2024-25

- [Final project for the Mobile Robotics course - A.A. 2024-25](#final-project-for-the-mobile-robotics-course---aa-2024-25)
  - [Unity environment](#unity-environment)
  - [ROS workspace](#ros-workspace)
    - [Topics](#topics)
    - [Example: publish commands to move an obstacle](#example-publish-commands-to-move-an-obstacle)
    - [Example: run the obstacle detection/avoidance algorithm on the robot](#example-run-the-obstacle-detectionavoidance-algorithm-on-the-robot)
- [Utils](#utils)

## Unity environment

- **Unity Simulation**: load the project inside /unityStuffs

## ROS workspace

> Turtlebots 3 hardcoded trajectories for unity simulation can be found in the folder `/ros2_ws/src/mob_rob_obstacle_avoidance`

### Topics  

| topic               | type                                                                                                   | description              |
| ------------------- | ------------------------------------------------------------------------------------------------------ | ------------------------ |
| /cmdvel             | [geometry_msgs/Twist Message](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html)               |   robot command |
| /cmdvel_obstacle{n} | [geometry_msgs/Twist Message](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html)               | n-th obstacle trajectory |
| /robot_odom         | [nav_msgs/msg/Odometry Message](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)             |                          |
| /robot_lidar   | [sensor_msgs/LaserScan Message](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) |                          |
| /robot_camera_l     | [sensor_msgs/msg/Image Message](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html)             |                          |
| /robot_camera_r     | [sensor_msgs/msg/Image Message](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html)             |                          |
| /robot_camera_depth              | [sensor_msgs/msg/Image Message](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html)             |                          |


### Example: publish commands to move an obstacle

```sh
ros2 run mob_rob_obstacle_avoidance move_obstacle --ros-args -p trajectory:="rectangle" -p obstacle_index:=1
```

### Example: run the obstacle detection/avoidance algorithm on the robot

```sh
ros2 run mob_rob_obstacle_avoidance control_robot
```

# Utils

```sh
ros2 topic pub /topic std_msgs/String 'data: Hello World' -t 1000
```