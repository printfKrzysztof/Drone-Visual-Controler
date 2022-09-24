#include <ros/ros.h>
#include <dvc_msgs/drop_ball.h>
#include <mavros_msgs/OverrideRCIn.h>

#define DROP_A 1
#define DROP_B 2
#define NOTHING_TO_DROP 0
#define SERVO_DROP_CHANNEL 6
#define MID_SERVO 1500
#define A_SERVO_POS 1100
#define B_SERVO_POS 1900

int ball_to_drop = NOTHING_TO_DROP;

bool ball_drop_cb(dvc_msgs::drop_ball::Request& req, dvc_msgs::drop_ball::Response& res){
    if(req.ball_to_drop == "A"){
        ROS_INFO("Drop A");
        ball_to_drop = DROP_A;
        res.status = "Dropping A";
    } else {
        ROS_INFO("Drop B");
        ball_to_drop = DROP_B;
        res.status = "Dropping B";
    }
    return true;
}



int main(int argc, char** argv){
    ros::init(argc, argv, "ball_droper");
    ros::NodeHandle ball_droper("~");
    ros::ServiceServer ball_drop_srv = ball_droper.advertiseService("ball_drop", ball_drop_cb);
    ros::Publisher override_pub = ball_droper.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

    ros::Rate rate(5);
    mavros_msgs::OverrideRCIn overrideRcIn;
    for (int i = 0; i < 8; ++i) {
        overrideRcIn.channels[i] = mavros_msgs::OverrideRCIn::CHAN_NOCHANGE;
    }
    ROS_INFO("Ready to drop ball");
    while (ros::ok()){
        ros::spinOnce();
        switch (ball_to_drop) {
            case NOTHING_TO_DROP:
                overrideRcIn.channels[SERVO_DROP_CHANNEL] = mavros_msgs::OverrideRCIn::CHAN_NOCHANGE;
                override_pub.publish(overrideRcIn);
                break;
            case DROP_A:
                overrideRcIn.channels[SERVO_DROP_CHANNEL] = A_SERVO_POS;
                override_pub.publish(overrideRcIn);
                ros::spinOnce();
                ros::Duration(1).sleep();
                overrideRcIn.channels[SERVO_DROP_CHANNEL] = MID_SERVO;
                override_pub.publish(overrideRcIn);
                ros::spinOnce();
                ball_to_drop = NOTHING_TO_DROP;
                break;
            case DROP_B:
                overrideRcIn.channels[SERVO_DROP_CHANNEL] = B_SERVO_POS;
                override_pub.publish(overrideRcIn);
                ros::Duration(1).sleep();
                overrideRcIn.channels[SERVO_DROP_CHANNEL] = MID_SERVO;
                override_pub.publish(overrideRcIn);
                ros::spinOnce();
                ball_to_drop = NOTHING_TO_DROP;
                break;
            default:
                ball_to_drop = NOTHING_TO_DROP;
                break;
        }
        rate.sleep();
    }
    return 0;
}