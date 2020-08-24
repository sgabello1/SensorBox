#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import rospy
import argparse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

parser = argparse.ArgumentParser()
parser.add_argument("port", help="specify udp port")
args = parser.parse_args()


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = ('192.168.70.100', int(args.port))

def send(data):
    s.sendto(("husky," + str(data.data[:])).encode(), addr)

def main():
    rospy.init_node("huksyerr_send2udp", anonymous=True)
    rospy.Subscriber("/err_calc", Float32MultiArray, send)
    rospy.spin()


if __name__ == '__main__':
    main()
