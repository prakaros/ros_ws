#include <ros/ros.h>
#include "ros_romeo/RosRomeo.hpp"
#include <std_msgs/String.h>

namespace ros_romeo
{
    RosRomeo::RosRomeo(ros::NodeHandle& nodeHandle) : 
     nh_(nodeHandle)
    ,subscriberTopic_("/ros_juliet_node/Juliet")
    , loop_rate_param_(0)
    , romeo_num_param_(0)
    {
        subscriber_ = nh_.subscribe(subscriberTopic_, 100, &RosRomeo::readMessage, this);
        readParameters();
       // subscriber_ = nh_.subscribe<std_msgs::String>(subscriberTopic_, 100, boost::bind(&RosRomeo::readMessage, this, _1, "User1"));
    }

     RosRomeo::~RosRomeo()
     {
     
     }
     
     void RosRomeo::readMessage(const std_msgs::String::ConstPtr msg)
     {
        ROS_INFO_STREAM("Recvd message from Juliet "<< msg->data);
        ROS_INFO("Recvd msg from Juliet [%s] ", msg->data.c_str());
        if(loop_rate_param_)
        {
           ROS_INFO_STREAM("Romeo:: loop_rate "<< loop_rate_param_); 
        }
        
        if(romeo_num_param_)
        {
            ROS_INFO_STREAM("Romeo:: romeo number "<< romeo_num_param_);
        }
        
     }
     
     void RosRomeo::readParameters()
     {
        if(!nh_.getParam("loop_rate", loop_rate_param_))
        {
            ROS_ERROR("Could not find loop_rate param");
        }
        else
        {
            ROS_INFO_STREAM("Romeo:: loop_rate "<< loop_rate_param_);
        }
        

        if(!nh_.getParam("romeo_number", romeo_num_param_))
        {
            ROS_ERROR("Could not find romeo_number param");
        }
        else
        {
            ROS_INFO_STREAM("Romeo:: romeo_number "<< romeo_num_param_);
        }
        
     }
     
}
