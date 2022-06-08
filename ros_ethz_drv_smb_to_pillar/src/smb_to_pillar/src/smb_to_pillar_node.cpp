#include <ros/ros.h>
#include <smb_to_pillar/SmbToPillar.hpp>

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "smb_to_pillar");
    
	ROS_INFO_STREAM("Creating ROS node smb_to_pillar");
    std::cout<<"Creating ros node smb_to_pillar "<<std::endl;        
	ros::NodeHandle nodeHandle("~");
    std::cout<<"ROS::NodeHandle "<<std::endl;  
	ROS_INFO_STREAM("Created  ROS nodeHandle");
	
	smb_to_pillar::SmbToPillar smbToPillar(nodeHandle);
	
	ROS_INFO_STREAM("Created ROS SmbToPillar");
    std::cout<<"ROS::NodeHandle "<<std::endl;  
    //smbToPillar.execute();
	ros::spin();
    return 0;
}


