/**
 * @file read_yolo_data_functions.cpp
 * @author Krzysztof B (github)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */


#include <read_yolo_data.h>

ros::Publisher search_pub;
darknet_ros_msgs::BoundingBoxes boundingBoxes;
bool yolo_flag = false;

void yolo_sub_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    boundingBoxes = *msg;
    yolo_flag = true;
}

void InitPublisher(ros::NodeHandle controlnode)
{
    std::string ros_ns;
    if (!controlnode.hasParam("namespace"))
    {
        ros_ns = "";
    }
    else
    {
        controlnode.getParam("namespace", ros_ns);
    }
    controlnode.advertise<dvc_msgs::SearchResults>((ros_ns + "/read_yolo_data/search_results").c_str(), 10);
}

bool checkFlags()
{
    return yolo_flag;
}

void resetFlags()
{
    yolo_flag = 0;
}

void PublishMsg()
{
    dvc_msgs::SearchResults found_objects;
    dvc_msgs::SearchResult found_object;
    for (const auto &bounding_box_auto : boundingBoxes.bounding_boxes)
    {
        found_object.centre_x = (bounding_box_auto.xmax + bounding_box_auto.xmin) / 2;
        found_object.centre_y = (bounding_box_auto.ymax + bounding_box_auto.ymin) / 2;
        found_object.id = bounding_box_auto.id;
        found_object.size = (bounding_box_auto.xmax - bounding_box_auto.xmin) * (bounding_box_auto.ymax - bounding_box_auto.ymin);
        ROS_INFO((std::to_string(found_object.size) + "IS_OK?").c_str());

        found_objects.search_results.push_back(found_object);
    }
    search_pub.publish(found_objects);
    //TODO ?WHY NOT PUBLISHING?
}
