//
// Created by maciek on 22.05.2021.
//

#include "mission_commander_3colors.h"

sensor_msgs::NavSatFix global_position;
nav_msgs::Odometry local_position;
trajectory_planer_msgs::TrajectoryPlaner waypointToTree;
mavros_msgs::State mavState;
mavros_msgs::ExtendedState extendedMavState;
sensor_msgs::Image yoloImage;
bool needPhoto = false;

ros::Publisher set_local_pos_pub;
ros::Publisher set_heading_pub;
ros::Publisher set_offset_pub;
ros::Publisher set_mode_pub;
ros::Publisher set_global_pos_pub;
ros::Publisher waypoint_reach_pub;
ros::Publisher object_photo_pub;
ros::Publisher founded_object_pub;
ros::Publisher mission_start_pub;

ros::Subscriber global_pose_sub;
ros::Subscriber local_pose_sub;
ros::Subscriber trajectory_planer_sub;
ros::Subscriber mav_state_sub;
ros::Subscriber extended_mav_state_sub;
ros::Subscriber yolo_Photo_sub;

ros::ServiceClient ball_droper_client;


void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    global_position = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    local_position = *msg;
}

void trajectory_planer_cb(const trajectory_planer_msgs::TrajectoryPlaner::ConstPtr& msg){
    waypointToTree = *msg;
}
void mav_state_cb(const mavros_msgs::State::ConstPtr& msg){
    mavState = *msg;
}
void ext_mav_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){
    extendedMavState = *msg;
}

void yolo_photo_cb(const sensor_msgs::Image::ConstPtr& msg){
    if(needPhoto) yoloImage = *msg;
}


void init_publisher_subscriber(ros::NodeHandle controlNode){
    // Publishers
    object_photo_pub = controlNode.advertise<sensor_msgs::Image>("/mission_commander/object_photo", 1);
    set_local_pos_pub = controlNode.advertise<geometry_msgs::Point>("/drone_ridder/set_local_position", 1);
    set_heading_pub = controlNode.advertise<std_msgs::Float64>("/drone_ridder/set_heading", 1);
    set_offset_pub = controlNode.advertise<geometry_msgs::Point>("/drone_ridder/set_position_offset", 1);
    set_mode_pub = controlNode.advertise<std_msgs::String>("/drone_ridder/set_mode", 1);
    set_global_pos_pub = controlNode.advertise<geographic_msgs::GeoPoseStamped>("/drone_ridder/set_global_position", 1);
    waypoint_reach_pub = controlNode.advertise<trajectory_planer_msgs::TrajectoryPlaner>("/trajectory_planer/waypoint_reach", 1);
    //Subscribers
    global_pose_sub = controlNode.subscribe("/mavros/global_position/global", 1, global_pos_cb);
    local_pose_sub = controlNode.subscribe("/mavros/global_position/local", 1, local_pos_cb);
    trajectory_planer_sub = controlNode.subscribe("/trajectory_planer/next_waypoint", 1, trajectory_planer_cb);
    mav_state_sub = controlNode.subscribe("/mavros/state", 1, mav_state_cb);
    extended_mav_state_sub = controlNode.subscribe("/mavros/extended_state", 1, ext_mav_state_cb);
    yolo_Photo_sub = controlNode.subscribe("/darknet_ros/detection_image", 1, yolo_photo_cb);
    founded_object_pub = controlNode.advertise<std_msgs::String>("/mission_commander/founded_object", 1);
    mission_start_pub = controlNode.advertise<std_msgs::String>("/mission_commander/mission_start", 1);
    // Services
    //ball_droper_client = controlNode.serviceClient<ball_droper_msgs::drop_ball>("drop_ball");
}

MissionState startMission(sensor_msgs::NavSatFix* takeOffPointWGS84, nav_msgs::Odometry* takeOffPoint){
    // Wait until drone starts
    if (mavState.system_status != MAV_STATE_ACTIVE){
        ros::Duration(0.5).sleep();
        return MissionState::waitingForArm;
    }
    std_msgs::String msg;
    msg.data = "Mission Start";
    mission_start_pub.publish(msg);
    *takeOffPointWGS84 = global_position;
    *takeOffPoint = local_position;
    ROS_INFO("ARMED");
    return MissionState::scanField;
}
MissionState scanField(ros::NodeHandle controlNode, std::stack<sensor_msgs::NavSatFix>* waypoints){
    sensor_msgs::NavSatFix waypoint;
    //geographic_msgs::GeoPoseStamped waypointOut;
    double waypointPositionAccuracy, aslToWGS83, flyAlt;
    controlNode.getParam("/3color/waypointPositionAccuracy", waypointPositionAccuracy);
    controlNode.getParam("/3color/aslToWGS83", aslToWGS83);
    controlNode.getParam("/3color/flyAlt", flyAlt);
    ROS_INFO("number of waypoints: %lu", waypoints->size());
    if (!waypoints->empty()){
        waypoint = waypoints->top();
        ROS_INFO("Distance to point: %f", pointDistance(globalToLocalPosition(waypoint, controlNode)));
        if(pointDistance(globalToLocalPosition(waypoint, controlNode)) > waypointPositionAccuracy){
            std_msgs::Float64 heading;
            heading.data = headingToPoint(globalToLocalPosition(waypoint, controlNode));
            //waypointOut.pose.position.longitude = waypoint.longitude;
            //waypointOut.pose.position.latitude = waypoint.latitude;
            //waypointOut.pose.position.altitude = waypoint.altitude + aslToWGS83;
            set_global_pos_pub.publish(globalPosToSend(waypoint, aslToWGS83));
            set_heading_pub.publish(heading);
        } else {
            waypoints->pop();
            ROS_INFO("Scan field: waypoint reached");
        }
        return MissionState::scanField;
    }
    return MissionState::goToNextObject;
}

MissionState goToNextObject(ros::NodeHandle controlNode){
    double waypointPositionAccuracy;
    double lowFlyAlt;
    double aslToWGS83;
    controlNode.getParam("/3color/waypointPositionAccuracy", waypointPositionAccuracy);
    controlNode.getParam("/3color/lowFlyAlt", lowFlyAlt);
    controlNode.getParam("/3color/aslToWGS83", aslToWGS83);
    geometry_msgs::Point waypoint;
    waypoint.x = waypointToTree.pos1;
    waypoint.y = waypointToTree.pos2;
    waypoint.z = lowFlyAlt;
    if(waypointToTree.mode == "empty"){
            ROS_INFO("No more trees, going home");
            return MissionState::goHome;
    }
    if(waypointToTree.updateCounter < 5){
        waypoint_reach_pub.publish(waypointToTree);
        return MissionState::goToNextObject;
    }
    if(pointDistance(waypoint) > waypointPositionAccuracy){
        set_global_pos_pub.publish(globalPosToSend(localToGlobalPosition(waypoint), aslToWGS83));
        //set_local_pos_pub.publish(waypoint);
        //ROS_INFO("going to %f, %f, %f", waypoint.x, waypoint.y, waypoint.z);
    } else{
        ROS_INFO("Tree reach: x: %f, y: %f", waypoint.x, waypoint.y);
        ros::Duration(1).sleep();
        return MissionState::takeCloseLook;
    }
    return MissionState::goToNextObject;
}

MissionState takeCloseLook(ros::NodeHandle controlNode){
    double closeLookAlt, waypointPositionAccuracy;
    double aslToWGS83;
    controlNode.getParam("/3color/closeLookAlt", closeLookAlt);
    controlNode.getParam("/3color/waypointPositionAccuracy", waypointPositionAccuracy);
    controlNode.getParam("/3color/aslToWGS83", aslToWGS83);
    geometry_msgs::Point waypoint;
    waypoint.x = waypointToTree.pos1;
    waypoint.y = waypointToTree.pos2;
    waypoint.z = closeLookAlt;
    set_global_pos_pub.publish(globalPosToSend(localToGlobalPosition(waypoint),aslToWGS83));
    if(pointDistance(waypoint) > waypointPositionAccuracy) {
        return MissionState::takeCloseLook;
    } else {
        trajectory_planer_msgs::TrajectoryPlaner waypointToTreeOld = waypointToTree;
        needPhoto = true;
        ros::spinOnce();
        ros::Duration(3).sleep();
        ros::spinOnce();
        if(waypointToTreeOld.updateCounter + 10 < waypointToTree.updateCounter){
            waypoint_reach_pub.publish(waypointToTreeOld);
            return MissionState::goToNextObject;
        }
        std_msgs::String objectClass_msg;
        if (waypointToTree.idClassObject == 1){
            objectClass_msg.data = "triangle";
        } else if(waypointToTree.idClassObject == 2){
            objectClass_msg.data = "square";
        } else {
            objectClass_msg.data = "circle";
        }
        founded_object_pub.publish(objectClass_msg);
        waypoint_reach_pub.publish(waypointToTree);
        object_photo_pub.publish(yoloImage);
        needPhoto = false;
        ros::spinOnce();
        return MissionState::goToNextObject;
    }

}

MissionState goHome(){
    if(mavState.mode != mavros_msgs::State::MODE_APM_COPTER_RTL){
        std_msgs::String mode;
        mode.data = "rtl";
        set_mode_pub.publish(mode);
        return MissionState::goHome;
    }
    if(extendedMavState.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND){
        ROS_INFO("Landed");
        return MissionState::standby;
    } else{
        ros::Duration(1).sleep();
    }
    return MissionState::goHome;
}

sensor_msgs::NavSatFix localToGlobalPosition(const geometry_msgs::Point& local){
    sensor_msgs::NavSatFix global;
    global.latitude = ((local.y - local_position.pose.pose.position.y) * MERES_TO_LATITUDE) + global_position.latitude;
    global.longitude = ((local.x - local_position.pose.pose.position.x) * (360.0 / (40075000 * cos(global_position.latitude * M_PI / 180)))) + global_position.longitude;
    global.altitude = local.z + global_position.altitude - local_position.pose.pose.position.z;
    return global;
}

geometry_msgs::Point globalToLocalPosition(const sensor_msgs::NavSatFix& global, const ros::NodeHandle& controlNode){
    geometry_msgs::Point local;
    double aslToWGS83;
    controlNode.getParam("/3color/aslToWGS83", aslToWGS83);
    local.z = global.altitude - global_position.altitude + local_position.pose.pose.position.z; // + aslToWGS83;
    local.x = local_position.pose.pose.position.x + (global.longitude - global_position.longitude) / (360.0 / 40075000 / cos(global_position.latitude * M_PI / 180));
    local.y = local_position.pose.pose.position.y + (global.latitude - global_position.latitude) / MERES_TO_LATITUDE;
    //ROS_INFO("LOCAL Z: %f, GLOBAL alt: %f, global_position: %f, local position: %f", local.z, global.altitude, global_position.altitude, local_position.pose.pose.position.z);
    return local;
}

geographic_msgs::GeoPoseStamped globalPosToSend(const sensor_msgs::NavSatFix& positionIn, const double& aslToWGS83){
    geographic_msgs::GeoPoseStamped positionOut;
    positionOut.pose.position.latitude = positionIn.latitude;
    positionOut.pose.position.longitude = positionIn.longitude;
    positionOut.pose.position.altitude = positionIn.altitude + aslToWGS83;
    return positionOut;
}
double pointDistance(const geometry_msgs::Point& destinationPoint){
    double dx = destinationPoint.x - local_position.pose.pose.position.x;
    double dy = destinationPoint.y - local_position.pose.pose.position.y;
    double dz = destinationPoint.z - local_position.pose.pose.position.z;
    return (sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2)));
}

double pointDistance(const sensor_msgs::NavSatFix& dest){
    double dla = dest.latitude - global_position.latitude;
    double dlo = dest.longitude - global_position.longitude;
    double dalt = dest.altitude - global_position.altitude;
    double dx = dla / MERES_TO_LATITUDE;
    double dy = dlo / (360.0 / 40075000 / cos(global_position.latitude * M_PI / 180));
    double dz = dalt;
    ROS_INFO("error pos: %f, %f, %f", dx, dy, dz);
    return (sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2)));
}

double headingToPoint(const geometry_msgs::Point& destinationPoint){
    double dx = destinationPoint.x - local_position.pose.pose.position.x;
    double dy = destinationPoint.y - local_position.pose.pose.position.y;
    double angle = atan2(dy, dx);
    double heading;
    if (angle >= 0){
        heading = angle / M_PI * 180;
    } else{
        heading = (-angle / M_PI * 180) + 180;
    }
    return heading;
}

MissionState generateWaypoints(std::stack<sensor_msgs::NavSatFix>* waypoints, ros::NodeHandle controlNode){
    if (global_position.altitude == 0) return MissionState::generateWaypoints;
    double pointALo, pointALa, pointBLo, pointBLa, pointCLo, pointCLa, pointDLo, pointDLa;
    int numberOfPaths;
    double flyAlt;
    controlNode.getParam("/3color/pointALo", pointALo);
    controlNode.getParam("/3color/pointALa", pointALa);
    controlNode.getParam("/3color/pointBLo", pointBLo);
    controlNode.getParam("/3color/pointBLa", pointBLa);
    controlNode.getParam("/3color/pointCLo", pointCLo);
    controlNode.getParam("/3color/pointCLa", pointCLa);
    controlNode.getParam("/3color/pointDLo", pointDLo);
    controlNode.getParam("/3color/pointDLa", pointDLa);
    controlNode.getParam("/3color/numberOfPaths", numberOfPaths);
    controlNode.getParam("/3color/flyAlt", flyAlt);
    sensor_msgs::NavSatFix waypoint;
    std::stack<sensor_msgs::NavSatFix> waypointsReverse;
    waypoint.latitude = pointALa;
    waypoint.longitude = pointALo;
    waypoint.altitude = global_position.altitude + flyAlt;
    waypoints->push(waypoint);
    if (numberOfPaths < 1) numberOfPaths = 1;
    for (int i = 0; i < numberOfPaths; ++i) {
        if(!(i % 2)){
            // A to B
            waypoint.latitude = pointBLa + (((pointBLa - pointCLa) / numberOfPaths) * i);
            waypoint.longitude = pointBLo + (((pointBLo - pointCLo) / numberOfPaths) * i);
            waypoint.altitude = global_position.altitude + flyAlt;
            waypointsReverse.push(waypoint);
            waypoint.latitude = pointBLa + (((pointBLa - pointCLa) / numberOfPaths) * (i + 1));
            waypoint.longitude = pointBLo + (((pointBLo - pointCLo) / numberOfPaths) * (i + 1));
            waypoint.altitude = global_position.altitude + flyAlt;
            waypointsReverse.push(waypoint);
        } else{
            // B to A
            waypoint.latitude = pointALa + (((pointALa - pointDLa) / numberOfPaths) * i);
            waypoint.longitude = pointALo + (((pointALo - pointDLo) / numberOfPaths) * i);
            waypoint.altitude = global_position.altitude + flyAlt;
            waypointsReverse.push(waypoint);
            waypoint.latitude = pointALa + (((pointALa - pointDLa) / numberOfPaths) * (i + 1));
            waypoint.longitude = pointALo + (((pointALo - pointDLo) / numberOfPaths) * (i + 1));
            waypoint.altitude = global_position.altitude + flyAlt;
            waypointsReverse.push(waypoint);
        }
    }
    int waypointsNumber = 0;
    while (!waypointsReverse.empty()){
        waypoint = waypointsReverse.top();
        waypoints->push(waypoint);
        waypointsReverse.pop();
        waypointsNumber++;
    }
    ROS_INFO("Waypoints generated");
    return MissionState::waitingForArm;
}