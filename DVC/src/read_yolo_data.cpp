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

class YoloTranslator
{

private:

	dvc_msgs::SearchResult SingleObject;
	ros::Publisher pub;
	ros::Subscriber sub;

public:

	/**
	 * @brief Construct a new Yolo Translator object
	 * 
	 * @param nh NodeHandler
	 */
	YoloTranslator(ros::NodeHandle *nh)
	{
		std::string ros_ns;
		if (nh->hasParam("namespace"))
		{
			ros_ns = "";
		}
		else
		{
			nh->getParam("namespace", ros_ns);
		}
		pub = nh->advertise<dvc_msgs::SearchResults>((ros_ns + "/read_yolo_data/search_results").c_str(), 10);
		sub = nh->subscribe("/darknet_ros/bounding_boxes", 1000, &YoloTranslator::callback_number, this);
	}

	/**
	 * @brief Callback for found objects by darknet_ros
	 * 
	 * @param msg Our messege
	 */
	void callback_number(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)

	{
		dvc_msgs::SearchResults AllObjects;

		for (const auto &bounding_box_auto : msg->bounding_boxes)
		{
			SingleObject.centre_x = (bounding_box_auto.xmax + bounding_box_auto.xmin) / 2;
			SingleObject.centre_y = (bounding_box_auto.ymax + bounding_box_auto.ymin) / 2;
			SingleObject.id = bounding_box_auto.id;
			SingleObject.size = (bounding_box_auto.xmax - bounding_box_auto.xmin) * (bounding_box_auto.ymax - bounding_box_auto.ymin);
			ROS_INFO((std::to_string(SingleObject.size) + "IS_OK?").c_str());

			AllObjects.search_results.push_back(SingleObject);
		}

		pub.publish(AllObjects);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "read_yolo_data");
	ros::NodeHandle read_yolo_data;
	YoloTranslator yt = YoloTranslator(&read_yolo_data);
	ros::spin();
}