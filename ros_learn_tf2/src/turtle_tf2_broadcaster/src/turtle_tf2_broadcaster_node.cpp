#include <ros/ros.h>
#include "turtle_tf2_broadcaster/Tft2Broadcaster.hpp"

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "turtle_tf2_broadcaster");
    ROS_INFO_STREAM("Creating turtle-tf2-broadcaster Turtle:: " << argv[1]);
    ros::NodeHandle nodeHandle("~");
    tf2_broad_caster::Tf2Broadcaster tf2BrNodeHandler_(nodeHandle, argv[1]);
    ros::spin();
    return 0;
}


