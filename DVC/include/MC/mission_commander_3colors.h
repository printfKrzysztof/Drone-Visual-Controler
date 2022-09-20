//
// Created by maciek on 22.05.2021.
//
#pragma once

#include <ros/ros.h>
#include <trajectory_planer_msgs/TrajectoryPlaner.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <stack>
#include <sensor_msgs/Image.h>




#define MAV_STATE_ACTIVE 4
#define MERES_TO_LATITUDE 0.000008983031

enum class MissionState{
    standby,
    generateWaypoints,
    waitingForArm,
    scanField,
    goToNextObject,
    takeCloseLook,
    goHome
};

void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
void trajectory_planer_cb(const trajectory_planer_msgs::TrajectoryPlaner::ConstPtr& msg);
void mav_state_cb(const mavros_msgs::State::ConstPtr& msg);
void init_publisher_subscriber(ros::NodeHandle controlNode);

MissionState startMission(sensor_msgs::NavSatFix* takeOffPointWGS84, nav_msgs::Odometry* takeOffPoint);
MissionState scanField(ros::NodeHandle controlNode, std::stack<sensor_msgs::NavSatFix>* waypoints);
MissionState goToNextObject(ros::NodeHandle controlNode);
MissionState takeCloseLook(ros::NodeHandle controlNode);
MissionState goHome();
geometry_msgs::Point globalToLocalPosition(const sensor_msgs::NavSatFix& global, const ros::NodeHandle& controlNode);
sensor_msgs::NavSatFix localToGlobalPosition(const geometry_msgs::Point& local);
geographic_msgs::GeoPoseStamped globalPosToSend(const sensor_msgs::NavSatFix& positionIn, const double& aslToWGS83);
MissionState generateWaypoints(std::stack<sensor_msgs::NavSatFix>* waypoints, ros::NodeHandle controlNode);
double pointDistance(const geometry_msgs::Point& destinationPoint);
double headingToPoint(const geometry_msgs::Point& destinationPoint);

