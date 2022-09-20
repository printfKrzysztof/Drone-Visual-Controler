#pragma once

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>


/**
\defgroup control_functions
This module is designed to make high level control programming more simple. 
*/




void state_cb(const mavros_msgs::State::ConstPtr& msg);
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void global_pos_home_cb(const mavros_msgs::HomePosition::ConstPtr& msg);
void positionOffset_cb(const geometry_msgs::Point::ConstPtr& msg);
void positionGlobalSet_cb(const geographic_msgs::GeoPoseStamped::ConstPtr& msg);
void localPositionSet_cb(const geometry_msgs::Point::ConstPtr& msg);
void headingSet_cb(const std_msgs::Float64::ConstPtr& msg);
void modeChange_cb(const std_msgs::String::ConstPtr& msg);


geometry_msgs::Point get_current_location();
float get_current_heading();
sensor_msgs::NavSatFix get_current_pos_global();
void set_heading(float heading);
void set_local_destination(float x, float y, float z);
void set_global_destination(float latitude, float longitude, float altitude);
int wait4connect();
int wait4start();
int arm();
int takeoff(float takeoff_alt);
int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01);
int set_mode(const std::string& mode);
int land();
int set_speed(float speed_mps);
int init_publisher_subscriber(ros::NodeHandle controlnode);



