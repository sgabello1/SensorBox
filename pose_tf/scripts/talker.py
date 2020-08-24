#!/usr/bin/env python  
import roslib 
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance
import math
import time


if __name__ == '__main__':
    rospy.init_node('pose_tf')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    pose_tf = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
    trans = tfBuffer.lookup_transform('map', 'world',rospy.Time(0),rospy.Duration(1.0))
    wp1 = PoseWithCovariance()
    ## WP2
    

    wp1.pose.position.x = -3.41388380381
    wp1.pose.position.y = -5.04600568964
    wp1.pose.position.z = 0.421817075425

    wp1.pose.orientation.x = -0.00346357583061
    wp1.pose.orientation.y = 0.00427715047416
    wp1.pose.orientation.z = 0.803023547028
    wp1.pose.orientation.w = -0.595921895588
    wp1_transformed = tf2_geometry_msgs.do_transform_pose(wp1, trans)

    wp1_transformed_st = PoseWithCovarianceStamped() 
    wp1_transformed_st.pose.pose.position = wp1_transformed.pose.position
    wp1_transformed_st.pose.pose.orientation = wp1_transformed.pose.orientation
	
    pose_tf.publish(wp1_transformed_st)
    
    time.sleep(2)
    print "wp 2"

    ## WP3
    wp1.pose.position.x = 1.30165610587
    wp1.pose.position.y = -1.55171769059
    wp1.pose.position.z = 0.399708028339

    wp1.pose.orientation.x = -0.00184794586291
    wp1.pose.orientation.y = -0.00630307605557
    wp1.pose.orientation.z = -0.373115837911
    wp1.pose.orientation.w = -0.927761517461
    wp1_transformed = tf2_geometry_msgs.do_transform_pose(wp1, trans)

    wp1_transformed_st = PoseWithCovarianceStamped() 
    wp1_transformed_st.pose.pose.position = wp1_transformed.pose.position
    wp1_transformed_st.pose.pose.orientation = wp1_transformed.pose.orientation

    pose_tf.publish(wp1_transformed_st)

    time.sleep(1)
    print "wp 3"

    ## WP4
    wp1.pose.position.x = 4.9280696055
    wp1.pose.position.y = 2.44382647284
    wp1.pose.position.z = 0.382309819649

    wp1.pose.orientation.x = -0.00528022570207
    wp1.pose.orientation.y = -0.00525134096999
    wp1.pose.orientation.z = -0.44961072356
    wp1.pose.orientation.w = -0.893193526051
    wp1_transformed = tf2_geometry_msgs.do_transform_pose(wp1, trans)

    wp1_transformed_st = PoseWithCovarianceStamped() 
    wp1_transformed_st.pose.pose.position = wp1_transformed.pose.position
    wp1_transformed_st.pose.pose.orientation = wp1_transformed.pose.orientation

    pose_tf.publish(wp1_transformed_st)
    time.sleep(1)
    print "wp 4"

    ## WP5
    wp1.pose.position.x = 6.9542160447
    wp1.pose.position.y = 7.47259858989
    wp1.pose.position.z = 0.36011276478

    wp1.pose.orientation.x = -0.000672673033611
    wp1.pose.orientation.y = -0.00533124213512
    wp1.pose.orientation.z = -0.585323444295
    wp1.pose.orientation.w = -0.810781978347
    wp1_transformed = tf2_geometry_msgs.do_transform_pose(wp1, trans)

    wp1_transformed_st = PoseWithCovarianceStamped() 
    wp1_transformed_st.pose.pose.position = wp1_transformed.pose.position
    wp1_transformed_st.pose.pose.orientation = wp1_transformed.pose.orientation

    pose_tf.publish(wp1_transformed_st)
    time.sleep(1)
    print "wp 5"


    ## WP6
    wp1.pose.position.x = 0.776374921509
    wp1.pose.position.y = 3.66423974718
    wp1.pose.position.z = 0.389573285844

    wp1.pose.orientation.x = 0.000283530799746
    wp1.pose.orientation.y = -0.00341586477001 
    wp1.pose.orientation.z = -0.961037223533
    wp1.pose.orientation.w = 0.276394572148
    wp1_transformed = tf2_geometry_msgs.do_transform_pose(wp1, trans)

    wp1_transformed_st = PoseWithCovarianceStamped() 
    wp1_transformed_st.pose.pose.position = wp1_transformed.pose.position
    wp1_transformed_st.pose.pose.orientation = wp1_transformed.pose.orientation

    pose_tf.publish(wp1_transformed_st)
    time.sleep(1)
    print "wp 6"

    ## WP7
    wp1.pose.position.x = -0.132179206119
    wp1.pose.position.y = 6.05237869124
    wp1.pose.position.z = 0.386123696176

    wp1.pose.orientation.x = -0.00189409887785
    wp1.pose.orientation.y = -0.0067007693862
    wp1.pose.orientation.z = -0.97150126787
    wp1.pose.orientation.w = -0.236930474742
    wp1_transformed = tf2_geometry_msgs.do_transform_pose(wp1, trans)

    wp1_transformed_st = PoseWithCovarianceStamped() 
    wp1_transformed_st.pose.pose.position = wp1_transformed.pose.position
    wp1_transformed_st.pose.pose.orientation = wp1_transformed.pose.orientation

    pose_tf.publish(wp1_transformed_st)
    time.sleep(1)
    print "wp 7"



    ## WP8
    wp1.pose.position.x = -2.80205473105
    wp1.pose.position.y = 1.68235251603
    wp1.pose.position.z = 0.405481320111

    wp1.pose.orientation.x = 0.00676155918898
    wp1.pose.orientation.y = 0.000718424039614
    wp1.pose.orientation.z = -0.607098149908
    wp1.pose.orientation.w = 0.794597732245
    wp1_transformed = tf2_geometry_msgs.do_transform_pose(wp1, trans)

    wp1_transformed_st = PoseWithCovarianceStamped() 
    wp1_transformed_st.pose.pose.position = wp1_transformed.pose.position
    wp1_transformed_st.pose.pose.orientation = wp1_transformed.pose.orientation

    pose_tf.publish(wp1_transformed_st)
    time.sleep(5)
    print "wp 8"

    
    ## WP1

    wp1.pose.position.x = -1.95693401441
    wp1.pose.position.y = -1.87900204426
    wp1.pose.position.z = 0.405331625728

    wp1.pose.orientation.x = 0.000190102473902
    wp1.pose.orientation.y = 0.00258721671434
    wp1.pose.orientation.z = 0.727793822143
    wp1.pose.orientation.w = -0.685791071677
    wp1_transformed = tf2_geometry_msgs.do_transform_pose(wp1, trans)

    wp1_transformed_st = PoseWithCovarianceStamped() 
    wp1_transformed_st.pose.pose.position = wp1_transformed.pose.position
    wp1_transformed_st.pose.pose.orientation = wp1_transformed.pose.orientation

    pose_tf.publish(wp1_transformed_st)
    time.sleep(1)
    print "wp 1"

