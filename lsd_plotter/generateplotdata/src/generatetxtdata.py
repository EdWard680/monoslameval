#!/usr/bin/env python2

# Extract images from a bag file.

# PKG = 'beginner_tutorials'
import roslib;  # roslib.load_manifest(PKG)
import os
import rosbag
import rospy
import cv2
import pylab
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
from itertools import islice
import sys

bag_file = "LSD_pose_machine.bag"#"morning.bag"#"LSD_pose2.bag"#"morning_work.bag"


def readDataFormBag():
    bag = rosbag.Bag(bag_file, "r")
    info = bag.get_type_and_topic_info()
    bag_data = bag.read_messages('/lsd_slam/pose')  #bag.read_messages('/lsd_slam/pose')  #bag.read_messages('slam_pose')
    fhandle = open("/home/chendh/Rosbag_for_plotting/generated_txt/lsd_pose_machine.txt", 'w')  # /home/chendh/Rosbag_for_plotting/generated_txt/slam_pose.txt
    for topic, msg, t in bag_data:
        timestr = "%.6f" %  msg.header.stamp.to_sec()
        #print(timestr)
        x_temp = msg.pose.position.x
        y_temp = msg.pose.position.y
        z_temp = msg.pose.position.z
        qx_temp = msg.pose.orientation.x
        qy_temp = msg.pose.orientation.y
        qz_temp = msg.pose.orientation.z
        qw_temp = msg.pose.orientation.w
        list_temp = [x_temp, y_temp, z_temp, qx_temp, qy_temp, qz_temp, qw_temp]


        fhandle.write(str(timestr))
        fhandle.write(' ')
        for i in range(len(list_temp)):
            fhandle.write(str(list_temp[i]))
            if i < 6:
                fhandle.write(' ')
        fhandle.write('\n')

    fhandle.close()


if __name__ == "__main__":
    readDataFormBag()