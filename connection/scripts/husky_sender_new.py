#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import rospy
import argparse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
import tf2_ros




class HuskySender(object):
    def __init__(self, ip, port):
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.ip = ip
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (str(ip), int(port))
        rospy.Subscriber("/err_calc", Float32MultiArray, self.errSender)
        rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, self.pathSender)

    def errSender(self, data):
        self.s.sendto(("husky," + str(data.data[:])).encode(), self.addr)

    def pathSender(self, data):
        try:
            odom = self.tfBuffer.lookup_transform("world", "odom", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        husky_path = []
        husky_path.append([odom.transform.translation.x, odom.transform.translation.y, odom.transform.translation.z,
                           odom.transform.rotation.x, odom.transform.rotation.y, odom.transform.rotation.z, odom.transform.rotation.w])
        for pose in data.poses:
            husky_path.append([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                               pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])

        self.s.sendto(("path," + str(husky_path)).encode(), self.addr)




if __name__ == '__main__':
    rospy.init_node("husky_sender", anonymous=True)
    huskySender = HuskySender("192.168.1.114", '60000')
    rospy.spin()
