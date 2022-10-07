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

std::string saved_namespace;
uint8_t hasNs=0;
void SaveNamespace(ros::NodeHandle controlnode)
{
    std::string ros_namespace;
    if (!controlnode.hasParam("namespace"))
    {
        hasNs=1;
    }
    else
    {
        hasNs=2;
    controlnode.getParam("namespace", saved_namespace);
    }
    
}
std::string GetNamespace()
{
    while(hasNs==0)
    {

    }
    if (hasNs==1) return "";
    else return saved_namespace;
}

#endif // COMMON_DATA_HPP