#include <ros/ros.h>
#include "static_broadcast_tf2/StaticBroadcaster.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "static_broadcaster");
    ROS_INFO_STREAM("Creating static-broadcaster ");
    ros::NodeHandle nodeHandle_("~");
    static_broad_caster::StaticBroadcaster tf2sb_(nodeHandle_, argv[1]);
    if(!tf2sb_.useParameters())
    {
        //Use commandline arguments
        tf2sb_.broadcaster_static_tf2(argv[2], argv[3], argv[4], argv[5], argv[6], argv[7]);
    }
    else
    {
        //Use parameters from launch file
        tf2sb_.execute();
    }
    ros::spin();
    return 0;

}
