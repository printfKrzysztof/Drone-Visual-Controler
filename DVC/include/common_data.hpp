/**
 * @file common_data.hpp
 * @author Krzysztof B (github)
 * @brief
 * @version 0.1
 * @date 2022-10-04
 *
 * @cite IQ_SIM
 *
 */

#ifndef COMMON_DATA_HPP
#define COMMON_DATA_HPP

#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <dvc_msgs/SearchResults.h>
#include <string>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#define FLAG_NEW_MESSAGE 0b00000001
#define FLAG_USER_LOCKED 0b00000010

#endif // COMMON_DATA_HPP