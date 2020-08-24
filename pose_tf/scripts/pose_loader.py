#!/usr/bin/env python  
import roslib 
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
import math
import time

rospy.init_node('pose_tf')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
pose_tf = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
trans = tfBuffer.lookup_transform('map', 'world',rospy.Time(0),rospy.Duration(1.0))

text_file = open("waypoints.txt", "r")
lines = text_file.readlines()
wp = PoseWithCovariance()
wp_transformed_st = PoseWithCovarianceStamped()

for i in range(0,len(lines)):
	el = lines[i].split(',')
	wp.pose.position.x = float(el[0])
	wp.pose.position.y = float(el[1])
	wp.pose.position.z = float(el[2])

	wp.pose.orientation.x = float(el[3])
	wp.pose.orientation.y = float(el[4])
	wp.pose.orientation.z = float(el[5])
	wp.pose.orientation.w = float(el[6].split('\n')[0])
	
	wp_transformed = tf2_geometry_msgs.do_transform_pose(wp, trans)
	wp_transformed_st.pose.pose.position = wp_transformed.pose.position
        wp_transformed_st.pose.pose.orientation = wp_transformed.pose.orientation
        pose_tf.publish(wp_transformed_st)
        time.sleep(2)
	#print wp.pose.orientation.w #Debug
	print "WP " , i

text_file.close()
