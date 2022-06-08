#include <ros/ros.h>
#include "turtle_tf2_listener/TurtleTf2Listener.hpp"

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "turtle_tf2_listener");
    ROS_INFO_STREAM("Creating ROS node turtle_tf2_listener");
    ros::NodeHandle nodeHandle_("~");
    ROS_INFO_STREAM("Created  ROS nodeHandle");
    turtle_tf2_listener::TurtleTf2Listener turtleTfListener_(nodeHandle_);
    ROS_INFO_STREAM("Created ROS TurtleTf2Listener");
    turtleTfListener_.execute();
    return 0;
}
