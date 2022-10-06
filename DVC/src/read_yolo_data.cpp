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

#include <read_yolo_data.hpp>

int checked =0;
void yolo_sub_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
	int j;
	for (int i = 0; i < msg->bounding_boxes.size(); i++) // For all discovered Boxes
	{
		if(msg->bounding_boxes[i].Class == "kite" && msg->bounding_boxes[i].probability>0.1)
		{
			ROS_INFO("lol");
		}
	}
}

int main(int argc, char **argv)
{
	//TODO
	ros::init(argc, argv, "YOLO2DVC");
	ros::NodeHandle nsub;
	//ros::NodeHandle npub("~");
	ros::Subscriber sub = nsub.subscribe("/darknet_ros/bounding_boxes", 1, yolo_sub_callback);
	//init_search_publisher(npub,checked);
	checked=1;
	ros::spin();
	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */

	return 0;
}
