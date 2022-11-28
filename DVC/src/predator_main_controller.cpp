/**
 * @file prey_control.cpp
 * @author Krzysztof B (github)
 * @brief
 * @version 0.1
 * @date 2022-10-01
 *
 * @cite IQ_SIM
 *
 */

// Includes
#include <gnc_functions.hpp>
#include <main.hpp>
#include <math.h>
#include <kalman.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

extern uint8_t flags;
gnc_api_waypoint WayPoint;
geometry_msgs::Point a3dpoint;
std_msgs::String data_to_pilot;

// Discrete LTI projectile motion, measuring position only

void detection_cb(const dvc_msgs::SearchResults::ConstPtr &msg)
{

    // ROS_INFO("Recived detection");
    for (const auto &search_result_auto : msg->search_results)
    {
        if (!(flags & FLAG_USER_LOCKED))
        {
            data_to_pilot.data.clear();
            data_to_pilot.data.append("Wykryto obiekt");
            data_to_pilot.data.append("  --> Press <- to lock in \n");
            flags |= FLAG_FROM_YOLO_M;
        }
        else
        {
            a3dpoint = get_current_location();
            WayPoint.psi = get_current_heading();
            WayPoint.x = a3dpoint.x + search_result_auto.distance_prediction * sin(M_PI / 180 * (search_result_auto.angle - WayPoint.psi)); // s)earch_result_auto.distance_prediction;
            WayPoint.y = a3dpoint.y + search_result_auto.distance_prediction * cos(M_PI / 180 * (search_result_auto.angle - WayPoint.psi));
            WayPoint.z = a3dpoint.z + search_result_auto.height_correction;
            WayPoint.psi = get_current_heading() - search_result_auto.angle;
        }
    }
}

void pilot_cb(const std_msgs::Char::ConstPtr &msg)
{
    char recived_cmd = msg->data;
    std::string temp(&recived_cmd, 1);
    ROS_INFO((temp + "recived").c_str());
    switch (recived_cmd)
    {
    case 'a':
        flags |= FLAG_USER_LOCKED;
        break;
    case 'b':
        break;
    }
}

// Variables
DVC_STATE state;
int main(int argc, char **argv)
{
    int n = 3; // Number of states
    int m = 1; // Number of measurements

    double dt = 1.0 / 30; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd C(m, n); // Output matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance
    A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
    C << 1, 0, 0;

    // Reasonable covariance matrices
    Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
    R << 5;
    P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
    // initialize ros
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");
    // initialize control publisher/subscribers
    init_publisher_subscriber(gnc_node);
    // wait for FCU connection
    wait4connect();

    // wait for used to switch to mode GUIDED
    wait4start();

    // create local reference frame
    initialize_local_frame();

    ROS_INFO("PREDATOR INITIALIZED - READY TO FLY");

    ros::Subscriber sub = gnc_node.subscribe("/predator/read_yolo_data/search_results", 1, detection_cb);
    ros::Subscriber sub2 = gnc_node.subscribe("/Pilot", 1, pilot_cb);
    ros::Publisher pub_to_pilot = gnc_node.advertise<std_msgs::String>("/predator/ToPilot", 1);

    ros::Rate rate(4.0);
    int counter = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        if (flags & FLAG_FROM_YOLO_M)
        {
            // ROS_INFO("Msg published");
            pub_to_pilot.publish(data_to_pilot);
            flags &= !FLAG_FROM_YOLO_M;
        }
        switch (state)
        {
        case DVC_STATE_TAKEOFF:
            ROS_INFO_ONCE("Taking off");
            // request takeoff
            takeoff(3);
            state = DVC_STATE_SEARCH;
            break;

        case DVC_STATE_SEARCH:
            /* code */
            ROS_INFO_ONCE("Searching");
            if (flags & FLAG_USER_LOCKED)
            {
                state = DVC_STATE_FOLLOW_AND_CAPTURE;
            }
            set_speed(30);
            break;

        case DVC_STATE_FOLLOW_AND_CAPTURE:
            ROS_INFO_ONCE("FOLLOWING TO PREDICT POINTS");
            set_destination(WayPoint.x, WayPoint.y, WayPoint.z, WayPoint.psi);
            // set_heading(WayPoint.psi);
            break;

        case DVC_STATE_AIM:
            break;

        case DVC_STATE_LANDING:
            land();
            break;

        case DVC_STATE_LOST:
            break;

        default:
            break;
        }
    }
    return 0;
}