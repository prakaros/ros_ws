#include <ros/ros.h>
#include "ros_romeo/RosRomeo.hpp"

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "ros_romeo");
    ros::NodeHandle rosNodelHandle("~");
    ros_romeo::RosRomeo rosRomeo(rosNodelHandle);
    ros::spin();
    return 0;

}

