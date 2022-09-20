#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#define ASL_TO_WGS83 -33.5 // Łódź
//#define ASL_TO_WGS83 -37 // Kąkolewo

sensor_msgs::NavSatFix global_position;
nav_msgs::Odometry local_position;

void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    global_position = *msg;
    //ROS_INFO("get global pose");
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    local_position = *msg;
    //ROS_INFO("get global pose");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_DR");
    ros::NodeHandle test_DR_square("~");
    std::string rosNamespace = test_DR_square.getNamespace();
    ros::Publisher set_local_pos_pub = test_DR_square.advertise<geometry_msgs::Point>(
            "/drone_ridder/set_local_position", 1);
    ros::Publisher set_heading_pub = test_DR_square.advertise<std_msgs::Float64>("/drone_ridder/set_heading", 1);
    ros::Publisher set_offset_pub = test_DR_square.advertise<geometry_msgs::Point>("/drone_ridder/set_position_offset",
                                                                                   1);
    ros::Publisher set_mode_pub = test_DR_square.advertise<std_msgs::String>("/drone_ridder/set_mode", 1);
    ros::Publisher set_global_pos_pub = test_DR_square.advertise<geographic_msgs::GeoPoseStamped>(
            "/drone_ridder/set_global_position", 1);
    ros::Subscriber global_pose_sub = test_DR_square.subscribe("/mavros/global_position/global", 1, global_pos_cb);
    ros::Subscriber local_pose_sub = test_DR_square.subscribe("/mavros/global_position/local", 1, local_pos_cb);


    ros::Rate rate(1);
    geometry_msgs::Point waypoint;
    std_msgs::Float64 heading;
    geometry_msgs::Point positionOffset;
    std_msgs::String flightMode;
    geographic_msgs::GeoPoseStamped positionGlobal;
    float timeToSleep = 6;

    ROS_INFO("Get ready");
    //ROS_INFO("Test of set_mode");
    //flightMode.data = "takeoff"

    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();

    ROS_INFO("Test of set_global_position drone will fly 5m square");
    positionGlobal.pose.position.latitude = global_position.latitude;
    positionGlobal.pose.position.longitude = global_position.longitude;
    positionGlobal.pose.position.altitude = global_position.altitude + ASL_TO_WGS83;
    heading.data = 0;

    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint 1");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude,
             positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude,
             global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionGlobal.pose.position.latitude += 0.00005; // ~5m
    //positionGlobal.pose.position.longitude += 0;
    heading.data = 0;
    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint 2");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude,
             positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude,
             global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionGlobal.pose.position.longitude += (360.0 / 40075000 /
                                               cos(positionGlobal.pose.position.latitude * 3.1415 / 360) * 5.0); // ~5m;
    heading.data = 90;
    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint 3");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude,
             positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude,
             global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();


    positionGlobal.pose.position.latitude -= 0.00005; // ~5m
    heading.data = 180;
    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint 4");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude,
             positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude,
             global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionGlobal.pose.position.longitude -= (360.0 / 40075000 /
                                               cos(positionGlobal.pose.position.latitude * 3.1415 / 360) * 5.0); // ~5m;
    heading.data = 270;
    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint 5");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude,
             positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude,
             global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    ROS_INFO("End of test");
}
