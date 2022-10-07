/**
 * @file read_yolo_data.h
 * @author Krzysztof B (github)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef READ_YOLO_DATA_H
#define READ_YOLO_DATA_H

#include <common_data.hpp>

void resetFlags();
bool checkFlags();
void InitPublisher(ros::NodeHandle controlnode);
void yolo_sub_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
void PublishMsg();
#endif // !READ_YOLO_DATA_H