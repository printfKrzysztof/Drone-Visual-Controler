#pragma once

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
//#include <std_msgs/Int8.h>

#include <dvc_msgs/ObjectGlobalPosition.h>
#include <dvc_msgs/ObjectsGlobalPositions.h>
#include <dvc_msgs/TrajectoryPlaner.h>
#include <dvc_msgs/SimpleTree.h>
#include <dvc_msgs/treeTable.h>
#include <cmath>
#include <TP/TreeObejctPosition.h>
#include <vector>



void init_publisher(ros::NodeHandle controlNode);
void new_Point_cb(const dvc_msgs::ObjectsGlobalPositions::ConstPtr& msg);
void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
void achieve_point_cb(const dvc_msgs::TrajectoryPlaner::ConstPtr& msg);
bool tree_table_cb(dvc_msgs::treeTable::Request& req, dvc_msgs::treeTable::Response& res);
void resetReadFlag();
bool checkReadFlag();
void processReadPoints();
void findLoverCost (const std::vector<Point>& points, std::vector<size_t>& v, double actualCost, double& minCost, std::vector<size_t>& result);
std::vector<size_t> findBestTrajectory(const std::vector<Point>& points, const Point& dronePos);
void findTrajectory(ros::NodeHandle controlNode);
void sendOutMessage();
void printInfo();
void resetGoolFlag();
void setVisitedPoint();
