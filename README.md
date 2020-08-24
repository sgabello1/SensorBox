# 2D Autonomous Navigation with SensorBox (Velodyne + Husky Navigation Stack)

I know that with such an amazing set of sensors, as is SensorBox, performing 2D navigation with the traditional Husky navigation stack is definetly not mind blowing. But as we had problems in figuring out the initial best strategy to achieve a robust 2D navigation with SensorBox, I want to write down my learnings on this post in order to help others who are facing my same difficulties.

# SensorBox

SensorBox is a set of sensors binded togheter in Shanghai Jiao Tong University and BeiDou Research Institute that aims at performing indoor autonomous navigation by fusing the information retrieved by each sensor (sensor fusion).
SesorBox is composed of:
- 1 embedded computer (Ubuntu 16.04 ROS Kinetic) 
- 1 Velodyne 3D LiDAR
- 2 cameras
- 1 IMU

In this post we will focus on the traditional 2D navigation and we will hack Velodyne in order to make it working as it was a traditional 2D laser scanner (e.g RPLidar A3, Hokuyo etc).

<iframe width="560" height="315" src="https://www.youtube.com/embed/JbJeOqDqstU" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# STEPS
Assuming we have ROS up and running, first we need to install all the Husky navigation stack on our embedded computer. While installing the ROS packages is quite straightforward

``` sudo apt-get install ros-kinetic-husky-navigation ros-kinetic-husky-bringup ros-kinetic-husky-control ros-kinetic-husky-viz ```

we need to install also the husky-core service as [in this post in the wiki ROS](http://wiki.ros.org/husky_bringup/Tutorials/Install%20Husky%20Software%20%28Advanced%29) and the catch is that obviously you don't need to install all the Husky Ubuntu image from scratch but, as in the quoted tutorial, by creating and configuring properly the robot-wide setup file the husky-core service will work as a charm.

Now the trick is to transform the 3D Lidar Velodyne in a normal 2D laser scanner - I know a big waste of data... but it works!! So as in [this post](https://github.com/MengGuo/Jackal_Velodyne_Duke/tree/master/navigation) we install ``` pointcloud_to_laserscan ``` package from this [repo](https://github.com/ros-perception/pointcloud_to_laserscan) and same as in Jackal_Velodyne_Duke github post we edit the launch file "sample_node.launch"

``` 
<launch>
<node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">

  <remap from="cloud_in" to="/velodyne_points"/>
  <remap from="scan" to="/front/scan"/>
  <rosparam>
      transform_tolerance: 0.01
      min_height: 0.25
      max_height: 0.75

      angle_min: -3.1415
      angle_max: 3.1415
      angle_increment: 0.01
      scan_time: 0.1
      range_min: 0.9
      range_max: 130
      use_inf: true
      concurrency_level: 0
  </rosparam>

</node>
</launch>

``` 

while obviously adjusting the ROS topics (for me the topic /scan works perfectly on Rviz).

Finally we adjust the transformation from Velodyne to BaseLink by adding a static transformation to our "sample_node.launch" file 

``` <node pkg="tf" type="static_transformation_publisher" name="velodyne_to_front_laser" args="0 0 0 0 0 0" /base_link /velodyne 100" /> ``` 

and we are good to go!

# RUN THE CODE

When you start the robot make sure:
- the ethernet connection is established with Velodyne (No Internet connection)
- plug in the USB HUB in order to connect SensorBox to Husky and the Bluetooth dongle 
- unplug all other USB such as WIFI dongle etc (I have noticed that otherwise the computer becomes really slow)

Making the map with Gmapping:

``` sudo service husky_core stop ```

``` sudo service husky_core start```
(it's good to restart  Husky drivers)

``` roslaunch velodyne_pointcloud VLP16_points.launch ```

``` roslaunch pointcloud_to_laserscan sample_node.launch ```

``` roslaunch husky_navigation gmapping_demo.launch ```

``` roslaunch husky_viz view_robot.launch ```


Using AMCL, I have created a launch file to make it easier: 

``` roslaunch demo_launch sbox_demo_launch.launch ```

Have fun!
