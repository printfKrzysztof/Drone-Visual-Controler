#! /usr/bin/env python

import pymongo
import sys
import cv2
import rospy
import datetime
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, NavSatStatus, CameraInfo
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from trajectory_planer_msgs.msg import TrajectoryPlaner
import tf
# Credentials will be sent to you by e-mail
USER = 'team_491'
PASSWORD = 'kZVAVxVd6ON0'

VEHICLE_ID = 1
# Host IP and port will be sent to you by e-mail
HOST = "195.216.97.232"
PORT = 27027

# By default the user database has the same name as the user, if something changes we will let you know.
DATABASE = USER

latitude = 0
longitude = 0
altitude = 0

def connect_to_database():
    """
    This function is used to connect user to MongoDB database.
    :return: MongoDB database client, authenticated MongoDB database
    """
    try:
        client = pymongo.MongoClient(host=HOST, port=PORT)
        database = client[DATABASE]
        database.authenticate(USER, PASSWORD)
        return client, database

    except pymongo.errors.ConfigurationError:
        print("An Invalid URI host error was received. Is your Atlas host name correct in your connection string?")
        sys.exit(1)


client, database = connect_to_database()

def img2bytes(cv_image):
    """
    Load an image from the path and convert to bytes type.
    :param path: Path to an image file
    :return: img converted to bytes type
    """
    print("Img shape:\t", cv_image.shape)
    img = cv2.imencode('.jpg', cv_image)[1].tobytes()
    return img



def gps_rpy(vehicle_type: str, vehicle_id: int,
            gps_latitude: float, gps_longitude: float, gps_altitude: float,
            roll: float, pitch: float, yaw: float):
    """
    This function is used to return completed dictionary with location (GPS) and orientation (if its possible).

    :param vehicle_type: [string] select the type of vehicle,
        Choose these values: ["ugv", "uav"]
        ugv for driving vehicles,
        uav for drones or other flying vehicles
    :param vehicle_id: [int]
    :param gps_latitude: [float] Geographic coordinate of the center of the drone (latitude)
        Latitude [degrees]. Positive is north of equator; negative is south.
    :param gps_longitude: [float] Geographic coordinate of the center of the drone (longitude)
        Longitude [degrees]. Positive is east of prime meridian; negative is west.
    :param gps_altitude: [float] AGL - Above ground level -  flight altitude to terrain (altitude)
    :param roll: [float] Angle defining drone's orientation - roll (angle in radians).
    :param pitch: [float] Angle defining drone's orientation - pitch (angle in radians).
    :param yaw: [float] Angle defining drone's orientation - yaw (angle in radians).
    :return: [dict] completed dictionary
    """
    database['gps_rpy'].insert_one(dict(vehicle_type=vehicle_type,
                vehicle_id=vehicle_id,
                gps_latitude=gps_latitude,
                gps_longitude=gps_longitude,
                gps_altitude=gps_altitude,
                roll=roll,
                pitch=pitch,
                yaw=yaw))


def col_shape(shape: str, color: str, surface: float,
              gps_latitude: float, gps_longitude: float, img: bytes):
    """
    This function is used to return completed dictionary with data from col_shape.
    :param shape: [string] Shape of figure.
        Choose these values: ["circle", "square", "triangle"]
    :param color: [string] Color of figure.
        Choose these values: ["bronze", "beige", "gold"]
    :param surface: [float] Surface of figure, must be greater than or equal to 0
    :param gps_latitude: [float] Geographic coordinate of the center of the area (latitude)
        Latitude [degrees]. Positive is north of equator; negative is south.
    :param gps_longitude: [float] Geographic coordinate of the center of the area (longitude)
        Longitude [degrees]. Positive is east of prime meridian; negative is west.
    :param img: [bytes] Photography
    :return: [dict] completed dictionary
    """
    database['col_shape'].insert_one(dict(shape=shape,
                color=color,
                surface=surface,
                gps_latitude=gps_latitude,
                gps_longitude=gps_longitude,
                picture=img))


def col_sosnowsky(circle_surface: float, square_surface: float, triangle_surface: float,
                  gps_latitude: float, gps_longitude: float, img: bytes):
    """
    This function is used to return completed dictionary with data from col_sosnowsky.
    :param triangle_surface: [float] Surface of figure, must be greater than or equal to 0
    :param square_surface: [float] Surface of figure, must be greater than or equal to 0
    :param circle_surface: [float] Surface of figure, must be greater than or equal to 0
    :param gps_latitude: [float] Geographic coordinate of the center of the area (latitude)
        Latitude [degrees]. Positive is north of equator; negative is south.
    :param gps_longitude: [float] Geographic coordinate of the center of the area (longitude)
        Longitude [degrees]. Positive is east of prime meridian; negative is west.
    :param img: [bytes] Photography
    :return: [dict] completed dictionary
    """
    database['col_sosnowsky'].insert_one(dict(circle_surface=circle_surface,
                square_surface=square_surface,
                triangle_surface=triangle_surface,
                gps_latitude=gps_latitude,
                gps_longitude=gps_longitude,
                picture=img))


def tol_map(img: bytes):
    """
    This function is used to return completed dictionary with data from tol_map.
    :param img: [bytes] Photography
    :return: [dict] completed dictionary
    """
    database['tol_map'].insert_one(dict(picture=img))


def tol_detect(pathogen_type: str, gps_latitude: float, gps_longitude: float, img: bytes):
    """
    This function is used to return completed dictionary with data from tol_detect.
    :param pathogen_type: [string] Type of pathogen.
        Choose these values: ["bronze", "beige", "gold"]
    :param gps_latitude: [float] Geographic coordinate of the center of the area (latitude)
        Latitude [degrees]. Positive is north of equator; negative is south.
    :param gps_longitude: [float] Geographic coordinate of the center of the area (longitude)
        Longitude [degrees]. Positive is east of prime meridian; negative is west.
    :param img: [bytes] Photography
    :return: [dict] completed dictionary
    """
    database['tol_detect'].insert_one(dict(pathogen_type=pathogen_type,
                gps_latitude=gps_latitude,
                gps_longitude=gps_longitude,
                picture=img))


def tol_apply(pesticides_type: str, gps_latitude: float, gps_longitude: float):
    """
    This function is used to return completed dictionary with data from tol_apply.
    :param pesticides_type: [string] Type of spray application used.
        Choose these values: ["yellow", "orange"]
    :param gps_latitude: [float] Geographic coordinate of the center of the area (latitude)
        Latitude [degrees]. Positive is north of equator; negative is south.
    :param gps_longitude: [float] Geographic coordinate of the center of the area (longitude)
        Longitude [degrees]. Positive is east of prime meridian; negative is west.
    :return: [dict] completed dictionary
    """
    database['tol_apply'].insert_one(dict(pesticides_type=pesticides_type,
                gps_latitude=gps_latitude,
                gps_longitude=gps_longitude))


def start():
    """
    This function is used to return completed dictionary with time of signalling the start of the mission.
    :return: [dict] completed dictionary
    """
    database['start'].insert_one(dict(date=datetime.datetime.now()))


def finish():
    """
    This function is used to return completed dictionary with time of signalling the end of the mission.
    :return: [dict] completed dictionary
    """
    database['finish'].insert_one(dict(date=datetime.datetime.now()))

pose_initialized = False
euler = (0,0,0)


def gps_callback(msg):
    if(not pose_initialized):
        return
    x_lat = msg.latitude
    y_long = msg.longitude
    z_alt = msg.altitude
    global latitude
    latitude = msg.latitude
    global longitude
    longitude = msg.longitude
    global altitude
    altitude = msg.altitude
    gps_rpy('uav', VEHICLE_ID, x_lat, y_long, z_alt, *euler)


def pose_callback(msg):
    global pose_initialized, euler
    pose_initialized = True
    pose = msg.pose
    quaternion = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)


def founded_object_cb(msg):
    global latitude
    global longitude
    tol_detect(msg.data, latitude, longitude, bytes("hellothere", 'utf-8'))
    if msg.data == "gold":
        tol_apply("yellow", latitude, longitude)
    else:
        tol_apply("orange", latitude, longitude)


def start_cb(msg):
    start()


def main():
    rospy.init_node("insert_data_to_db_trees")
    r = rospy.Rate(10)
    start_subscriber = rospy.Subscriber('/mission_commander/mission_start', String, callback=start_cb)
    gps_subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, callback=gps_callback)
    pose_subscriber = rospy.Subscriber('/mavros/local_position/local/pose', PoseStamped, callback=pose_callback)
    founded_object_subscriber = rospy.Subscriber('/mission_commander/founded_object', String, callback=founded_object_cb)
    connect_to_database()
    while not rospy.is_shutdown():
        r.sleep()
    finish()
    client.close()


if __name__ == '__main__':
    main()
