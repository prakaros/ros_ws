#include <ros/ros.h>
#include "ros_juliet/RosJuliet.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_juliet");
    ros::NodeHandle rosNodeHandle("~");
    ros_juliet::RosJuliet rosJuliet(rosNodeHandle);
    rosJuliet.execute();
    return 0;
}

