#include <iostream>
#include <ros/ros.h>
#include "smb_highlevel_controller/SmbHighLevelController.hpp"

int main(int argc, char*argv[])
{
	ros::init(argc, argv, "smb_highlevel_controller");
	ROS_INFO_STREAM("Creating ROS node smb_highlevel_controller");
        std::cout<<"Creating ros node smb_highlevel_controller "<<std::endl;        
	ros::NodeHandle nodeHandle("~");
        std::cout<<"ROS::NodeHandle "<<std::endl;  
	ROS_INFO_STREAM("Created  ROS nodeHandle");
	smb_high_lvl_ctrl::SmbHighLevelController smbHighLvlController(nodeHandle);
	ROS_INFO_STREAM("Created ROS SmbHighLevelController");
        std::cout<<"ROS::NodeHandle "<<std::endl;  
	ros::spin();
	return 0;
}
