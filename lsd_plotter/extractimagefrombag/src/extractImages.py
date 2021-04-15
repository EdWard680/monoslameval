#!/usr/bin/env python2

import rosbag
import roslib  # roslib.load_manifest(PKG)
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bag_file = 'morning.bag'#'full_house.bag'#'dining_room_exploration2.bag'
bag = rosbag.Bag(bag_file, "r")

bridge = CvBridge()
bag_data = bag.read_messages('/camera/compressed')
for topic, msg, t in bag_data:
    cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("Image window", cv_image)
    #cv_image_resized = cv2.resize(cv_image, (640, 480))  # Resize to fit input requirements
    timestr = "%.6f" %  msg.header.stamp.to_sec()
    image_name = "/home/chendh/LSD_datasets/morning/images_origin/" + timestr + ".png"#"/home/chendh/LSD_datasets/dining_room_exploration2/images_origin/" + timestr + ".png"
    cv2.imwrite(image_name, cv_image)
    cv2.waitKey(3)
