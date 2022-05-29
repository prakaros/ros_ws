#include <ros/ros.h>
#include "ros_juliet/RosJuliet.hpp"
#include <std_msgs/String.h>

namespace ros_juliet
{
    RosJuliet::RosJuliet(ros::NodeHandle& nodeHandle) : 
    publishTopic_("Juliet")
    , msgToSend_()
    , nodeHandle_(nodeHandle)
    , loopRate_(1)
    , numMsg_(0)
    , magic_num_(0)
    {
        publisher_ = nodeHandle_.advertise<std_msgs::String>(publishTopic_, 100);
        readParameters();
    }  

    RosJuliet::~RosJuliet()
    {
    
    }
    
    void RosJuliet::readParameters()
    {
        if(!nodeHandle_.getParam("magic_number", magic_num_))
        {
            ROS_ERROR("Juliet:: Could not find param magic_number");
        }
        else
        {
            ROS_INFO_STREAM("Juliet:: Magic number " << magic_num_);
        }
    }
    
    void RosJuliet::sendMessage(const std::string& msg)
    {
        std_msgs::String msgToSend;
        msgToSend.data = msg;
        ROS_INFO_STREAM("Sending message from Juliet " << msgToSend.data);
        publisher_.publish(msgToSend);
    }

    void RosJuliet::execute()
    {
        while(ros::ok())
        {
            std::stringstream ss;
            ss << "Hi Romeo! This is Juliet " << numMsg_;
            
            if(magic_num_)
            {
                ss << "Magic number " << magic_num_;
            }
            
            msgToSend_ = ss.str();
            sendMessage(msgToSend_);
            ros::spinOnce();
            numMsg_++;
            loopRate_.sleep();
        }
        
    }
}
