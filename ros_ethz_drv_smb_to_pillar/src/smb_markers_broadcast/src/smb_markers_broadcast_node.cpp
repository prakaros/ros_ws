#include <ros/ros.h>
#include "smb_markers_broadcast/SmbMarkersBroadcast.hpp"

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "smb_markers_broadcast");
    ROS_INFO_STREAM("Creating ROS node smb_markers_broadcast");
    ros::NodeHandle nodeHandle_("~");
    ros::Duration(3.0).sleep();
    ROS_INFO_STREAM("Created  ROS nodeHandle");
    
    smb_markers_br::SmbMarkersBroadcast smbBr_(nodeHandle_);
    ROS_INFO_STREAM("Created ROS SmbMarkersBroadcast");
    smbBr_.execute();
    return 0;
}

