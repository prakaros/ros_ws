#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "smb_to_pillar/SmbToPillar.hpp"

namespace smb_to_pillar
{
    SmbToPillar::SmbToPillar(ros::NodeHandle& rnh) :
    nodeHandle_(rnh)
    , subscribeTopic_("/scan")
    , publisherTopic_("/cmd_vel")
    , subscriber_()
    , loopRate_(1)     //This is in Hz
    , numMsg_(0)
    , angleError_(0)
    {
        subscriber_ = nodeHandle_.subscribe(subscribeTopic_, 100, &SmbToPillar::laserscanCallback, this);
        publisher_  = nodeHandle_.advertise<geometry_msgs::Twist>(publisherTopic_, 100);
        
    }

    SmbToPillar::~SmbToPillar()
    {
    
    }
    
    void SmbToPillar::laserscanCallback(const sensor_msgs::LaserScan& message)
    {
    	size_t numReadings = message.ranges.size();
    	size_t minReadingIndex = 0;
    	float minReading = message.range_max;
    	bool validMin = false;
    	
    	for(size_t i = 0; i < numReadings; i++)
    	{
    	    if(message.ranges[i] > message.range_min && message.ranges[i] < message.range_max)
		    {
		        validMin = true;
		        if(message.ranges[i] < minReading)
		        {
		            minReading = message.ranges[i];
		            minReadingIndex = i;
		        }
		    }
    	}
    	
        if(validMin)
		{
		    float actualAngleMin = message.angle_min + message.angle_increment * minReadingIndex;
		    float expectedAngleMin = 0.0;
		    float pillar_x = minReading*std::cos(actualAngleMin);
		    float pillar_y = minReading*std::sin(actualAngleMin);
		    
		    ROS_INFO_STREAM("Dist min " << minReading << ", MinIndex "<< minReadingIndex << ", NumReadings " << numReadings);
		    ROS_INFO_STREAM("Angle:: Min " << message.angle_min << ", Max " << message.angle_max <<", Inc " << message.angle_increment );
		   // ROS_INFO_STREAM("Range:: Min " << message.range_min << ", Max " << message.range_max );
		   // ROS_INFO_STREAM("Time:: Scan " << message.scan_time << ", Inc " << message.time_increment);
		   ROS_INFO_STREAM("Min angle min " << actualAngleMin << ", Pillar_X "<< pillar_x << ", Pillar_Y " << pillar_y);
		   ROS_INFO_STREAM("---------------------------------------------");
		   
		   angleError_ = (actualAngleMin - expectedAngleMin);
		   if(minReading > 0.88)
		   {
		        velControlSmb();
		   }
		}
    }
    
    void SmbToPillar::sendTwistCommandToSmb(double linear_vel_x, double linear_vel_y, double angular_vel)
    {
        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = linear_vel_x;
        twistMsg.linear.y = linear_vel_y;
        twistMsg.linear.z = 0;
        
        twistMsg.angular.x = 0;
        twistMsg.angular.y = 0;
        twistMsg.angular.z = angular_vel;
        
        ROS_INFO_STREAM("Twist cmd_vel v_x:: " << twistMsg.linear.x << " v_y:: "<<twistMsg.linear.y << \
        " v_z:: " << twistMsg.linear.z <<"w_x:: "<<twistMsg.angular.x <<"w_y:: "<<twistMsg.angular.y << "w_z:: "<<twistMsg.angular.z);
        ROS_INFO_STREAM("...................................................");
        publisher_.publish(twistMsg);
    }
    
    void SmbToPillar::execute()
    {
        while(ros::ok())
        {
            velControlSmb();
            ros::spinOnce();
            numMsg_++;
            loopRate_.sleep();
        }
    }
    
    void SmbToPillar::velControlSmb()
    {
        double Kp = 1.2;
        double angularVel = angleError_;
        double linearVel = 0.0;
        if(std::abs(angularVel) < 0.2)
        {
            angularVel = 0.0;
            linearVel = 7.0;
        }
        else
        {
            angularVel *= Kp; 
            linearVel = 0.0;
         }
            
         sendTwistCommandToSmb(linearVel, 0, angularVel);
    }
}
