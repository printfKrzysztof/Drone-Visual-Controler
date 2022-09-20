#include <mission_commander_3colors.h>

int main(int argc, char** argv){
    sensor_msgs::NavSatFix takeOffPointWGS84;
    nav_msgs::Odometry takeOffPoint;
    ros::init(argc, argv, "mission_commander_3colors");
    ros::NodeHandle mission_commander_3colors("~");
    init_publisher_subscriber(mission_commander_3colors);
    ros::Rate rate(2);
    MissionState missionState = MissionState::generateWaypoints;
    std::stack<sensor_msgs::NavSatFix> waypoints;

    ROS_INFO("Commander 3color Active");
    while (ros::ok()){
        ros::spinOnce();
        switch (missionState) {
            case MissionState::generateWaypoints:
                missionState = generateWaypoints(&waypoints, mission_commander_3colors);
                break;
            case MissionState::waitingForArm:
                missionState = startMission(&takeOffPointWGS84, &takeOffPoint);
                break;
            case MissionState::scanField:
                missionState = scanField(mission_commander_3colors, &waypoints);
                break;
            case MissionState::goToNextObject:
                missionState = goToNextObject(mission_commander_3colors);
                break;
            case MissionState::takeCloseLook:
                missionState = takeCloseLook(mission_commander_3colors);
                break;
            case MissionState::goHome:
                missionState = goHome();
                break;
            case MissionState::standby:
                ros::Duration(5).sleep();
                ROS_INFO("Standby");
            default: break;
        }
        rate.sleep();
    }
}