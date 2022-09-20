
#include <std_msgs/Float64.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_DR");
    ros::NodeHandle test_DR_square("~");
    std::string rosNamespace = test_DR_square.getNamespace();
    ros::Publisher set_heading_pub = test_DR_square.advertise<std_msgs::Float64>("/drone_ridder/set_heading", 1);
    ros::Rate rate(1);
    std_msgs::Float64 heading;
    float timeToSleep = 6;
    ROS_INFO("Get ready");
    heading.data = 0;
    for (int i = 0; i < 5; ++i) {
        set_heading_pub.publish(heading);
        ROS_INFO("set heading to %f", heading.data);
        heading.data += 90;
        ros::Duration(timeToSleep).sleep();
    }
    ROS_INFO("End of test");
}