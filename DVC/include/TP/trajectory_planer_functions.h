#pragma once

#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
//#include <std_msgs/Int8.h>

#include <object_global_localizator_msgs/ObjectGlobalPosition.h>
#include <object_global_localizator_msgs/ObjectsGlobalPositions.h>
#include <trajectory_planer_msgs/TrajectoryPlaner.h>
#include <trajectory_planer_msgs/SimpleTree.h>
#include <trajectory_planer_msgs/treeTable.h>
#include <cmath>
#include <TreeObejctPosition.h>
#include <vector>



void init_publisher(ros::NodeHandle controlNode);
void new_Point_cb(const object_global_localizator_msgs::ObjectsGlobalPositions::ConstPtr& msg);
void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);
void achieve_point_cb(const trajectory_planer_msgs::TrajectoryPlaner::ConstPtr& msg);
bool tree_table_cb(trajectory_planer_msgs::treeTable::Request& req, trajectory_planer_msgs::treeTable::Response& res);
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
