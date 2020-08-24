# SensorBox

SensorBox a set of sensors developed in Shanghai Jiao Tong University and BeiDou Research Institute for autonomous navigation.SesorBox is composed of: (list of the sensors).

# REQUISITES
(list what you need installed)

# RUN THE CODE

When you start the robot make sure:
- the ethernet connection is established with Velodyne (No Internet connection)
- plug in the USB HUB in order to connect SensorBox to Husky and the Bluetooth dongle
- unplug all other USB such as RPLiDARA3, WIFI dongle etc



''' sudo service husky_core stop '''

'' sudo service husky_core start ''

'' roslaunch velodyne_pointcloud VLP16_points.launch ''

'' roslaunch pointcloud_to_laserscan sample_node.launch ''

'' roslaunch husky_navigation gmapping_demo.launch ''

'' roslaunch husky_viz view_robot.launch ''

# REFERENCES
