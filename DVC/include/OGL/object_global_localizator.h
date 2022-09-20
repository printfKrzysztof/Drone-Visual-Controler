#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <object_global_localizator_msgs/ObjectGlobalPosition.h>
#include <object_global_localizator_msgs/ObjectsGlobalPositions.h>
#include <cmath>

#define CAMERA_X_MAX 1920
#define CAMERA_Y_MAX 1080
#define CAMERA_X_ANGLE 0.685696
#define CAMERA_Y_ANGLE 0.342848
#define CAMERA_X_CENTER 960
#define CAMERA_Y_CENTER 540
#define MERES_TO_LATITUDE 0.000008983031



void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
//void object_detector_cb(const std_msgs::Int8::ConstPtr& msg);
void bounding_boxes_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
void setup_camera_rotation(double pitch);
void init_publisher(ros::NodeHandle controlNode);
void localizeObjects(ros::NodeHandle controlNode);
void resetFlags();
bool checkFlags();
void setDroneRotationMatrix();
