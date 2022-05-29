#pragma once

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
namespace ros_romeo
{
    class RosRomeo
    {
        public:
            RosRomeo(ros::NodeHandle& nodeHandle);
            virtual ~RosRomeo();
            void readMessage(const std_msgs::String::ConstPtr msg);
        private:
            void readParameters();
        private:
            ros::NodeHandle nh_;
            std::string subscriberTopic_;
            ros::Subscriber subscriber_;
            int loop_rate_param_;
            int romeo_num_param_;

    };
}
