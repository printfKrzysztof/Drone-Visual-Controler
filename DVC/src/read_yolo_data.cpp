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

#include <common_data.hpp>
#include <read_yolo_data.hpp>
uint8_t data_queue;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "read_yolo_data");
	ros::NodeHandle read_yolo_data("~");
	ros::Subscriber sub = read_yolo_data.subscribe("/darknet_ros/bounding_boxes", 1, yolo_sub_callback);
	InitPublisher(read_yolo_data);

	ros::Rate rate(10.0);

	while (ros::ok())
	{

		ros::spinOnce();
		if (checkFlags())
		{
			PublishMsg();
			resetFlags();
		}
		rate.sleep();
	}

	return 0;
}
