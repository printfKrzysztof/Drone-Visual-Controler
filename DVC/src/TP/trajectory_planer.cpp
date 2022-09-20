#include <trajectory_planer_functions.h>



unsigned int errorCounter = 0;

int main(int argc, char** argv) {
    //initialize ros
    ros::init(argc, argv, "trajectory_planer");
    ros::NodeHandle trajectory_planer("~");

    ros::Subscriber point_sub = trajectory_planer.subscribe("/object_global_localizator", 1, new_Point_cb);
    ros::Subscriber global_pose_sub = trajectory_planer.subscribe("/mavros/global_position/global", 1, global_pos_cb);
    ros::Subscriber local_pose_sub = trajectory_planer.subscribe("/mavros/global_position/local", 1, local_pos_cb);
    ros::Subscriber achieve_point_sub = trajectory_planer.subscribe("/trajectory_planer/waypoint_reach", 1, achieve_point_cb);

    init_publisher(trajectory_planer);

    ros::ServiceServer tree_table_srv = trajectory_planer.advertiseService("/trajectory_planer/tree_table_srv", tree_table_cb);

    ros::Rate rate(2.0);
    ROS_INFO("Trajectory planer node started");
    resetReadFlag();
    resetGoolFlag();

    while(ros::ok())
    {
        ros::spinOnce();

        if(checkReadFlag()) //jesli sa dane o obiektach z object_global_localizator
        {
            processReadPoints(); //pobierz dane o obiektach i zaktualizuj liste wykrytych obiektow
            findTrajectory(trajectory_planer);
        }
        else
        {
            errorCounter++;
        }
        setVisitedPoint(); //oznacz obiekty juz odwiedzone
        sendOutMessage(); //publikuj wiadomosd o wykrytych obiektach

        printInfo(); //
    	if(checkReadFlag()) //jesli sa dane o obiektach z object_global_localizator
    	{
    		processReadPoints(); //pobierz dane o obiektach i zaktualizuj liste wykrytych obiektow
    		findTrajectory(trajectory_planer);
    	}
    	else
    	{
    		errorCounter++;
    	}
        setVisitedPoint(); //oznacz obiekty juz odwiedzone
    	sendOutMessage(); //publikuj wiadomosd o wykrytych obiektach

    	printInfo(); //


        resetReadFlag();
        rate.sleep();
    }
}