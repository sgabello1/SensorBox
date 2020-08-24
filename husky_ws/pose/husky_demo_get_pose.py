#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseStamped

import os
import argparse
from tf.transformations import euler_from_quaternion, quaternion_from_euler

pitch = yaw = roll =  0.0


def extract():

    rospy.init_node('get_husky_pose', anonymous=True)
    rospy.Subscriber('vrpn_client_node/husky/pose', PoseStamped, callback)
    rospy.spin()

def callback(Pose):
    n = 0
    f = open('./pose.txt', 'w')
    # f.write('timestamp,x,y,z,roll,pitch,yaw\n')
    x_list = []
    y_list = []
    z_list = []
    roll_list = []
    pitch_list = []
    yaw_list = []
    # for (topic, msg, ts) in bag.read_messages(topics='vrpn_client_node/husky/pose')):
    while (n < 500):
        orientation_list = [Pose.pose.orientation.x, Pose.pose.orientation.y, Pose.pose.orientation.z, Pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        x_list.append(Pose.pose.position.x)
        y_list.append(Pose.pose.position.y)
        z_list.append(Pose.pose.position.z)

	roll_list.append(roll)
	pitch_list.append(pitch)
	yaw_list.append(yaw)

        n += 1
        print(n)

    f.write('%.12f\n%.12f\n%.12f\n%.12f\n%.12f\n%.12f\n' %
            (
             sum(x_list)/len(x_list),
             sum(y_list)/len(y_list),
             sum(z_list)/len(z_list),
	     sum(pitch_list)/len(pitch_list),
	     sum(roll_list)/len(roll_list),
             sum(yaw_list)/len(yaw_list) - 1.57
             
             )
            )
    
    rospy.signal_shutdown("pose had write done!")
    print("pose had write done!")

if __name__ == '__main__':

    extract()
