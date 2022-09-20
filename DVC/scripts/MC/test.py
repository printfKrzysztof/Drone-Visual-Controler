#! /usr/bin/env python

import rospy
import tf

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String

latitude = 0
longitude = 0
altitude = 0
yaw = 0
pitch = 0
roll = 0


def cb1_fun(message):
    global latitude
    latitude = message.latitude
    global longitude
    longitude = message.longitude
    global altitude
    altitude = message.altitude


def cb2_fun(message):
    x = message.pose.pose.orientation.x
    y = message.pose.pose.orientation.y
    z = message.pose.pose.orientation.z
    w = message.pose.pose.orientation.w
    quaternion=(x,y,z,w)
    (roll_l, pitch_l, yaw_l)=tf.transformations.euler_from_quaternion(quaternion)
    global yaw
    yaw = yaw_l
    global pitch
    pitch = pitch_l
    global roll
    roll = roll_l


def cb3_fun(message):
    found_object = message
    if found_object == "Parch":
        rospy.loginfo("Parch")
    if found_object == "Maczniak":
        rospy.loginfo("Maczniak")







rospy.init_node("sub_gps")
sub1 = rospy.Subscriber('/mavros/global_position/global',  NavSatFix, cb1_fun)
sub2 = rospy.Subscriber('/mavros/global_position/local',  Odometry, cb2_fun)
sub3 = rospy.Subscriber('/mission_commander/founded_object',  String, cb3_fun)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    rospy.loginfo("latitiude: %f"%latitude + " longitude: %f"%longitude + " altitude: %f"%altitude + " Roll: %f"%roll + " Pitch: %f"%pitch + " Yaw: %f"%yaw)
    r.sleep()

