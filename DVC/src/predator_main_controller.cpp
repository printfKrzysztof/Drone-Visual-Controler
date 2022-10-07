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
#include <predator_main_controller.hpp>
#include <common_data.hpp>

void detection_cb(const dvc_msgs::SearchResults::ConstPtr &msg)
{
}

// Variables
DVC_STATE state;
int main(int argc, char **argv)
{
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
    // specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
    ros::Subscriber sub = gnc_node.subscribe( "predator/read_yolo_data/search_results", 1, detection_cb);
    ros::Rate rate(4.0);
    int counter = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        switch (state)
        {
        case DVC_STATE_TAKEOFF:
            ROS_INFO("Taking off");
            // request takeoff
            takeoff(3);
            state = DVC_STATE_SEARCH;
            break;

        case DVC_STATE_SEARCH:
            /* code */
            ROS_INFO_ONCE("Searching");
            break;
        case DVC_STATE_FOLLOW:
            /* code */
            break;
        case DVC_STATE_LANDING:
            /* code */
            break;
        default:
            break;
        }
    }
    return 0;
}