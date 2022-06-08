#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace smb_to_pillar
{
    class SmbToPillar
    {
        public:
            SmbToPillar(ros::NodeHandle& rnh_);
            virtual ~SmbToPillar();
            void laserscanCallback(const sensor_msgs::LaserScan& message);
            void sendTwistCommandToSmb(double linear_vel_x, double linear_vel_y, double angular_vel);
            void execute();
            void velControlSmb();
       private:
            ros::NodeHandle& nodeHandle_;
            std::string subscribeTopic_;
            std::string publisherTopic_;
            ros::Subscriber subscriber_;
            ros::Publisher publisher_;
            ros::Rate loopRate_;
            int numMsg_;
            float angleError_;
    };

}
