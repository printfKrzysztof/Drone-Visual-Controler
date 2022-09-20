//#include <drone_ridder.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>



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

int main(int argc, char** argv){
    ros::init(argc, argv, "test_DR");
    ros::NodeHandle test_DR_square("~");
    std::string rosNamespace = test_DR_square.getNamespace();
    ros::Publisher set_local_pos_pub = test_DR_square.advertise<geometry_msgs::Point>("/drone_ridder/set_local_position", 1);
    ros::Publisher set_heading_pub = test_DR_square.advertise<std_msgs::Float64>("/drone_ridder/set_heading", 1);
    ros::Publisher set_offset_pub = test_DR_square.advertise<geometry_msgs::Point>("/drone_ridder/set_position_offset", 1);
    ros::Publisher set_mode_pub = test_DR_square.advertise<std_msgs::String>("/drone_ridder/set_mode", 1);
    ros::Publisher set_global_pos_pub = test_DR_square.advertise<geographic_msgs::GeoPoseStamped>("/drone_ridder/set_global_position", 1);
    ros::Subscriber global_pose_sub = test_DR_square.subscribe("/mavros/global_position/global", 1, global_pos_cb);
    ros::Subscriber local_pose_sub = test_DR_square.subscribe("/mavros/global_position/local", 1, local_pos_cb);

    //global_pos_sub = controlnode.subscribe<sensor_msgs::NavSatFix>(ros_namespace + "/mavros/global_position/global", 10, global_pos_cb);

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

    ROS_INFO("Test of set_global_position");
    positionGlobal.pose.position.latitude = global_position.latitude;
    positionGlobal.pose.position.longitude = global_position.longitude;
    positionGlobal.pose.position.altitude = global_position.altitude;
    heading.data = 0;

    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint c1");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude, positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionGlobal.pose.position.latitude += 0.00005; // ~5m
    //positionGlobal.pose.position.longitude += 0;
    heading.data = 0;
    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint c2");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude, positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionGlobal.pose.position.longitude += (360.0 / 40075000 / cos(positionGlobal.pose.position.latitude * 3.1415 / 360) * 5.0); // ~5m;
    heading.data = 90;
    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint c3");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude, positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();


    positionGlobal.pose.position.latitude -= 0.00005; // ~5m
    heading.data = 180;
    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint c4");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude, positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionGlobal.pose.position.longitude -= (360.0 / 40075000 / cos(positionGlobal.pose.position.latitude * 3.1415 / 360) * 5.0); // ~5m;
    heading.data = 270;
    set_global_pos_pub.publish(positionGlobal);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint c5");
    ROS_INFO("dest: latitude %f; longitude %f; altitude %f", positionGlobal.pose.position.latitude, positionGlobal.pose.position.longitude, positionGlobal.pose.position.altitude);
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();


    ROS_INFO("Test of set_local_position and set_heading");
    waypoint.x = 0;
    waypoint.y = 0;
    waypoint.z = 10;
    heading.data = 0;
    set_local_pos_pub.publish(waypoint);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint a1");
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    waypoint.x = 5;
    heading.data = 0;
    set_local_pos_pub.publish(waypoint);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint a2");
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    waypoint.y = 5;
    heading.data = 90;
    set_local_pos_pub.publish(waypoint);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint a3");
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    waypoint.x = 0;
    heading.data = 180;
    set_local_pos_pub.publish(waypoint);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint a4");
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    waypoint.y = 0;
    heading.data = 270;
    set_local_pos_pub.publish(waypoint);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint a5");
    ROS_INFO("Current position latitude: %f longitude: %f altitude: %f ", global_position.latitude, global_position.longitude, global_position.altitude);
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    ROS_INFO("Test of set_position_offset");

    positionOffset.x = 0;
    positionOffset.y = 0;
    positionOffset.z = 0;
    heading.data = 0;
    set_offset_pub.publish(positionOffset);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint b1");
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionOffset.x = 5;
    positionOffset.y = 0;
    positionOffset.z = 0;
    heading.data = 0;
    set_offset_pub.publish(positionOffset);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint b2");
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionOffset.x = 0;
    positionOffset.y = 5;
    positionOffset.z = 0;
    heading.data = 90;
    set_offset_pub.publish(positionOffset);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint b3");
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionOffset.x = -5;
    positionOffset.y = 0;
    positionOffset.z = 0;
    heading.data = 180;
    set_offset_pub.publish(positionOffset);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint b4");
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionOffset.x = 0;
    positionOffset.y = -5;
    positionOffset.z = 0;
    heading.data = 270;
    set_offset_pub.publish(positionOffset);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint b5");
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();

    positionOffset.x = 0;
    positionOffset.y = 0;
    positionOffset.z = 0;
    heading.data = 0;
    set_offset_pub.publish(positionOffset);
    set_heading_pub.publish(heading);
    ROS_INFO("waypoint b6");
    ros::spinOnce();
    ros::Duration(timeToSleep).sleep();




    ROS_INFO("Test of set_mode");
    flightMode.data = "land";
    set_mode_pub.publish(flightMode);

}

