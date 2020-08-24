# 2D Autonomous Navigation with SensorBox

I know that with such an amazing set of sensors, as is SensorBox, performing 2D navigation with the traditional Husky navigation stack is definetly not mind blowing. But as we had problems in figuring out the initial best strategy to achieve a robust 2D navigation with SensorBox while extracting "more valuable" information from the other sensors, I ve decided to write down my learnings on this post.

# SensorBox

SensorBox is a set of sensors binded togheter in Shanghai Jiao Tong University and BeiDou Research Institute that aims at performing indoor autonomous navigation by fusing the information retrieved by each sensor (sensor fusion).
SesorBox is composed of:
- 1 embedded computer (Ubuntu 16.04 ROS Kinetic) 
- 1 Velodyne 3D LiDAR
- 2 cameras
- 1 IMU

In this post we will focus on the traditional 2D navigation and we will hack Velodyne in order to make it working as it was a traditional 2D laser scanner (e.g RPLidar A3, Hokuyo etc).

# STEPS
Assuming we have ROS up and running, first we need to install all the Husky navigation stack on our embedded computer. While installing the ROS packages is quite straightforward

``` sudo apt-get install ros-kinetic-husky-navigation ros-kinetic-husky-bringup ros-kinetic-husky-control ros-kinetic-husky-viz ```




# RUN THE CODE

When you start the robot make sure:
- the ethernet connection is established with Velodyne (No Internet connection)
- plug in the USB HUB in order to connect SensorBox to Husky and the Bluetooth dongle
- unplug all other USB such as RPLiDARA3, WIFI dongle etc



``` sudo service husky_core stop ```

``` sudo service husky_core start```

``` roslaunch velodyne_pointcloud VLP16_points.launch ```

``` roslaunch pointcloud_to_laserscan sample_node.launch ```

``` roslaunch husky_navigation gmapping_demo.launch ```

``` roslaunch husky_viz view_robot.launch ```

# REFERENCES
