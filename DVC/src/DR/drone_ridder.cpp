#include <drone_ridder.h>



int main(int argc, char** argv){
	//initialize ros 
	ros::init(argc, argv, "drone_ridder");
	ros::NodeHandle drone_ridder("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(drone_ridder);

	ros::Subscriber pos_offset_sub = drone_ridder.subscribe("/drone_ridder/set_position_offset", 1, positionOffset_cb);
	ros::Subscriber pos_global_sub = drone_ridder.subscribe("/drone_ridder/set_global_position", 1, positionGlobalSet_cb);
	ros::Subscriber pos_local_sub = drone_ridder.subscribe("/drone_ridder/set_local_position", 1, localPositionSet_cb);
	ros::Subscriber mode_sub = drone_ridder.subscribe("/drone_ridder/set_mode", 1, modeChange_cb);
    ros::Subscriber heading_sub = drone_ridder.subscribe("/drone_ridder/set_heading", 1, headingSet_cb);
  	// wait for FCU connection
	wait4connect();
	//wait for used to switch to mode GUIDED
	wait4start();
	//request takeoff
	takeoff(10);
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);

	while(ros::ok()){
		ros::spinOnce();
		//check_waypoint_reached();
		rate.sleep();
	}
	return 0;
}