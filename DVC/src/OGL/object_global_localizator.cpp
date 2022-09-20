#include <object_global_localizator.h>


unsigned int errorCounter = 0;

int main(int argc, char** argv) {
    //initialize ros
    ros::init(argc, argv, "object_global_localizator");
    ros::NodeHandle object_global_localizator("~");

    ros::Subscriber global_pose_sub = object_global_localizator.subscribe("/mavros/global_position/global", 1, global_pos_cb);
    ros::Subscriber local_pose_sub = object_global_localizator.subscribe("/mavros/global_position/local", 1, local_pos_cb);
    //ros::Subscriber YOLO_object_detector_sub = object_global_localizator.subscribe("darknet_ros/object_detector", 1, object_detector_cb);
    ros::Subscriber YOLO_bounding_boxes_sub = object_global_localizator.subscribe("/darknet_ros/bounding_boxes", 1, bounding_boxes_cb);
    init_publisher(object_global_localizator);
    setup_camera_rotation(M_PI);
    resetFlags();

    //ros::spin();
    ros::Rate rate(2.0);
    ROS_INFO("Localization Started");

    while(ros::ok())
    {
    	ros::spinOnce();

    	if(checkFlags())
    	{
    		//ROS_INFO("Condition passed");
    		setDroneRotationMatrix();
    		localizeObjects(object_global_localizator);
    	}
    	else
    	{
    		//ROS_INFO("%d",errorCounter);
    		errorCounter++;
    	}


    	resetFlags();
    	rate.sleep();
    }
}
