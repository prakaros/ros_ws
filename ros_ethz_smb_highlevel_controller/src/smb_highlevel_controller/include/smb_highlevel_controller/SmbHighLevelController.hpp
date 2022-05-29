#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>

namespace  smb_high_lvl_ctrl
{

    class SmbHighLevelController
    {
    	public:
    		SmbHighLevelController(ros::NodeHandle& nodeHandle);
            virtual ~SmbHighLevelController();
            void topicCallback(const sensor_msgs::LaserScan& message);      
            bool serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    	private:
    	    void readParameters();
    	private:
            ros::NodeHandle& nodeHandle_;
            std::string subscribeTopic_;
            ros::Subscriber subscriber_;
            ros::ServiceServer serviceServer_;
            std::string topic1_;
            std::string topic2_;
};


}

