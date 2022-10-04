/**
 * @file read_yolo_data.cpp
 * @author Krzysztof B (github)
 * @brief 
 * @version 0.1
 * @date 2022-10-04
 * 
 * @cite IQ_SIM
 * 
 */

#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

void yolo_sub_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for( int i=0; i < msg->bounding_boxes.size(); i++) //For all discovered Boxes
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
	}	

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "YOLO2DVC");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, yolo_sub_callback);
	ros::spin(); //makes main go in loop

	return 0;
}
