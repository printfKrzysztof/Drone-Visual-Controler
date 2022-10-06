/**
 * @file read_yolo_data.hpp
 * @author Krzysztof B (github)
 * @brief
 * @version 0.1
 * @date 2022-10-04
 *
 * @cite IQ_SIM
 *
 */

#ifndef READ_YOLO_DATA_HPP
#define READ_YOLO_DATA_HPP

#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <dvc_msgs/SearchResults.h>

// Publishers
ros::Publisher search_pub;

int init_search_publisher(ros::NodeHandle controlnode, int checked)
{
	if (checked!=0)
	{
		std::string ros_namespace;
		if (!controlnode.hasParam("namespace"))
		{

			ROS_INFO("using default namespace");
		}
		else
		{
			controlnode.getParam("namespace", ros_namespace);
			ROS_INFO("using namespace %s", ros_namespace.c_str());
		}

		search_pub = controlnode.advertise<dvc_msgs::SearchResults>((ros_namespace + "/read_data/search_results").c_str(), 10);
	}
	return 0;
}

#endif // READ_YOLO_DATA_HPP