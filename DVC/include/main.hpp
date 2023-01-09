/**
 * @file main.hpp
 * @author Krzysztof B (github)
 * @brief
 * @version 0.1
 * @date 2022-10-04
 *
 * @cite IQ_SIM
 *
 */

#ifndef MAIN_HPP
#define MAIN_HPP

#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <dvc_msgs/SearchResults.h>
#include <dvc_msgs/StatesVector.h>
#include <string>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#define FLAG_FROM_YOLO_M    0b00000001
#define FLAG_USER_LOCKED    0b00000010
#define FLAG_BUFFER_FULL    0b00000100

typedef enum
{

    DVC_STATE_TAKEOFF = 0,
    DVC_STATE_SEARCH,
    DVC_STATE_FOLLOW_AND_CAPTURE,
    DVC_STATE_AIM,
    DVC_STATE_LANDING,
    DVC_STATE_LOST,

} DVC_STATE;

uint8_t flags = 0;
#endif // MAIN_HPP