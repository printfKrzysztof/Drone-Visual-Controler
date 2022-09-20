#! /usr/bin/env python
# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import os

# Instantiate CvBridge
bridge = CvBridge()

image_counter = 0

Image_sub = None


def image_callback(msg):
    global image_counter
    image_counter += 1
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        path = '~/mission_photo'
        cv2.imwrite('{:10d}'.format(image_counter) + '.jpeg', cv2_img)


rospy.init_node('image_saver')

image_topic = "/jetbot_camera/raw"
# Set up your subscriber and define its callback
Image_sub = rospy.Subscriber(image_topic, Image, image_callback)

# Spin until ctrl + c
rospy.spin()
