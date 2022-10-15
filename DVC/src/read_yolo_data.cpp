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

#define MIDDLE_X 320
#define MIDDLE_Y 240
#define X_TO_RAD (float)(0.5 / 271)

class YoloTranslator
{

private:
	dvc_msgs::SearchResult SingleObject;
	ros::Publisher pub;
	ros::Subscriber yolo_sub;
	ros::Subscriber flag_sub;
	dvc_msgs::SearchResult Last10[10];
	uint8_t flags;

public:
	/**
	 * @brief Construct a new Yolo Translator object
	 *
	 * @param nh NodeHandler
	 */
	YoloTranslator(ros::NodeHandle *nh)
	{
		std::string ros_ns;
		if (!nh->hasParam("namespace"))
		{
			ros_ns = "";
		}
		else
		{
			nh->getParam("namespace", ros_ns);
		}
		pub = nh->advertise<dvc_msgs::SearchResults>((ros_ns + "/read_yolo_data/search_results").c_str(), 1);
		yolo_sub = nh->subscribe("/darknet_ros/bounding_boxes", 1, &YoloTranslator::yolo_callback, this);
		flag_sub = nh->subscribe((ros_ns + "/Flags").c_str(), 1, &YoloTranslator::flags_callback, this);
		ROS_INFO((ros_ns + " IS A NEW TOPIC").c_str());
	}

	/**
	 * @brief Callback for found objects by darknet_ros
	 *
	 * @param msg Our messege
	 */
	void yolo_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
	{
		dvc_msgs::SearchResults AllObjects;

		for (const auto &bounding_box_auto : msg->bounding_boxes)
		{

			int centre_x = (bounding_box_auto.xmax + bounding_box_auto.xmin) / 2;
			int centre_y = (bounding_box_auto.ymax + bounding_box_auto.ymin) / 2;
			// 271 -> 0,5 rad (90 deg)
			SingleObject.angle = (centre_x - MIDDLE_X) * X_TO_RAD;
			if (centre_y - 10 > MIDDLE_Y)
				SingleObject.height_correction = -1;
			else if (centre_y + 10< MIDDLE_Y)
				SingleObject.height_correction = 1;
			else
				SingleObject.height_correction = 0;

			SingleObject.distance_prediction = (bounding_box_auto.xmax - bounding_box_auto.xmin) * (bounding_box_auto.ymax - bounding_box_auto.ymin);

			AllObjects.search_results.push_back(SingleObject);
		}

		pub.publish(AllObjects);
	}

	void flags_callback(const std_msgs::UInt8::ConstPtr &msg)
	{
		flags = msg->data;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "read_yolo_data");
	ros::NodeHandle read_yolo_data("~");
	YoloTranslator yt = YoloTranslator(&read_yolo_data);
	ros::spin();
}