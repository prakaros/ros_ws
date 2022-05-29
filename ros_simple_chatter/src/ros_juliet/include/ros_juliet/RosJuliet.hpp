#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace ros_juliet
{
    class RosJuliet
    {
        public:
            RosJuliet(ros::NodeHandle& rnh);
            virtual ~RosJuliet();
            void execute();  
        private:
            void sendMessage(const std::string& msg);
            void readParameters();
        private:
            std::string publishTopic_;
            std::string msgToSend_;
            ros::NodeHandle nodeHandle_;
            ros::Publisher publisher_; 
            ros::Rate loopRate_;
            int numMsg_;
            int magic_num_;     
    };

}
