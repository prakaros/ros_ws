#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "smb_highlevel_controller/SmbHighLevelController.hpp"

namespace smb_high_lvl_ctrl
{
	SmbHighLevelController::SmbHighLevelController(ros::NodeHandle& nodeHandle) :
		nodeHandle_(nodeHandle)
		,subscribeTopic_("/scan")
	    ,subscriber_()
		,serviceServer_()
	{
	    readParameters();
		subscriber_ = nodeHandle_.subscribe(subscribeTopic_, 100, &SmbHighLevelController::topicCallback, this);
	}

	SmbHighLevelController::~SmbHighLevelController()
	{

	}

	void SmbHighLevelController::readParameters()
	{
        if(!nodeHandle_.getParam("topic_laser", topic1_))
        {
            ROS_ERROR("Failed to read param topic_laser ");
        }
        else
        {
            ROS_INFO_STREAM("topic_laser val" << topic1_);
        }
        
        if(!nodeHandle_.getParam("topic_odom", topic2_))
        {
            ROS_ERROR("Failed to read param topic_odom ");
        }
        else
        {
            ROS_INFO_STREAM("topic_odom val" << topic2_);
        }
        
	}
	
	void SmbHighLevelController::topicCallback(const sensor_msgs::LaserScan& message)
	{
		//ROS_INFO_STREAM("Rcvd laser scan AngleMax " << message.angle_max << " AngleMin:: "<<message.angle_min);
		//ROS_INFO_STREAM("Rcvd laser scan RangeMax " << message.range_max << " RangeMin:: "<<message.range_min);
		//ROS_INFO_STREAM("Number of points recvd " << message.ranges.size());
		std::vector<float> scanVal;
		
		for(auto elem : message.ranges)
		{
		    if(elem > message.range_min && elem < message.range_max)
		    {
		        scanVal.push_back(elem);
		    }
		}
		
		std::vector<float>::const_iterator result = std::min(scanVal.begin(), scanVal.end());
		if(result != scanVal.end())
		{
		    ROS_INFO_STREAM("Min distance " << *result);
		}
		
	}

    bool SmbHighLevelController::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
    {
    	return false;

    }
}
