#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import socket
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("port", help="specify udp port")
args = parser.parse_args()

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("192.168.1.114", int(args.port)))
print("UDP bound on port %s..." % args.port)


def main():
    rospy.init_node("server", anonymous=True)
    vo_pub = rospy.Publisher("vo_err", Float32MultiArray, queue_size=1)
    husky_pub = rospy.Publisher("husky_err", Float32MultiArray, queue_size=1)
    while not rospy.is_shutdown():
        data, addr = s.recvfrom(1024)
        print("Receive from %s:%s" % addr)
        msg = Float32MultiArray()
        data = data.replace("(", "").replace(")", "").split(",")
        msg.data = list(map(float, data[1:]) )
        if data[0] == "vo":
            vo_pub.publish(msg)
        if data[0] == "husky":
            husky_pub.publish(msg)

if __name__ == '__main__':
    main()
