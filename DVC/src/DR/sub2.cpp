#include <gnc_functions.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>


//include API 



void achieveGoal_cb(const std_msgs::Bool::ConstPtr& msg)
{	
	if(msg->data)
		ROS_INFO("Osiagnol cel");
	else
		ROS_INFO("w drodze");
}

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "sub2_node");
	ros::NodeHandle tree_seeker_node("~");

	ros::Subscriber sub = tree_seeker_node.subscribe("/tree_seeker/set_goal", 1, achieveGoal_cb);
	ros::spin();

	return 0;
}