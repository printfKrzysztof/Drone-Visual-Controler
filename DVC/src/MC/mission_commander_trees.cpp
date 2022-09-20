//
// Created by maciek on 22.05.2021.
//
#include <mission_commander_trees.h>

int main(int argc, char** argv){
    sensor_msgs::NavSatFix takeOffPointWGS84;
    nav_msgs::Odometry takeOffPoint;
    /*
    sensor_msgs::NavSatFix missionStartPoint;
    geometry_msgs::Point missionStartPointLocal;
    missionStartPointLocal.x = -40;
    missionStartPointLocal.y = 10;
    missionStartPointLocal.z = FLY_ALT;

    geometry_msgs::Point fieldEndPoint;
    fieldEndPoint.x = -40;
    fieldEndPoint.y = 90;
    fieldEndPoint.z = FLY_ALT;
    */
    ros::init(argc, argv, "mission_commander_trees");
    ros::NodeHandle mission_commander_trees("~");
    init_publisher_subscriber(mission_commander_trees);
    ros::Rate rate(2);
    MissionState missionState = MissionState::waitingForArm;
    ROS_INFO("Commander ACTIVE, mission TREE");
    while (ros::ok()){
        ros::spinOnce();
        switch (missionState) {
            case MissionState::waitingForArm:
                missionState = startMission(&takeOffPointWGS84, &takeOffPoint);
                break;
            case MissionState::gettingOnMissionStartPlace:
                //missionState = getToStartPlace(missionStartPoint, takeOffPointWGS84); //global option
                missionState = getToStartPlace(mission_commander_trees);
                break;
            case MissionState::firstLookAtField:
                missionState = firstLookAtField(mission_commander_trees);
                break;
            case MissionState::goToNextTree:
                missionState = goToNextObject(mission_commander_trees);
                break;
            case MissionState::dropBall:
                missionState = dropBall(mission_commander_trees);
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

    return 0;
}