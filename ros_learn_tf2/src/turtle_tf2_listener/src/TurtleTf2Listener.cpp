#include <ros/ros.h>
#include "turtle_tf2_listener/TurtleTf2Listener.hpp"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Spawn.h>

namespace turtle_tf2_listener
{
    TurtleTf2Listener::TurtleTf2Listener(ros::NodeHandle& rnh) : nodeHandle_(rnh)
    , cmdVel_()
    , tf2Buffer_()
    , tf2Listener_(tf2Buffer_)
    , tf2Stamped_()
    , turtleMaster_("turtle1")
    , turtleFollower_("turtle2")
    , loopRate_(10)
    , turtle1_()
    {
  #if 0
        ros::service::waitForService("spawn");
        
        spawner_ = nodeHandle_.serviceClient<turtlesim::Spawn>("spawn");
        
        turtle1_.request.x = 2.0;
        turtle1_.request.y = 4.0;
        turtle1_.request.theta = 0.0;
        turtle1_.request.name = turtleFollower_;
        spawner_.call(turtle1_);
  #endif      
        velPubFollower_ =  nodeHandle_.advertise<geometry_msgs::Twist>(turtleMaster_+"/cmd_vel", 100);
    }
    
    TurtleTf2Listener::~TurtleTf2Listener()
    {
    
    }
    
    void TurtleTf2Listener::spawnTurtle()
    {
        
    }
    
    void TurtleTf2Listener::execute()
    {
         while(nodeHandle_.ok())
         {
            try
            {
                tf2Stamped_ = tf2Buffer_.lookupTransform(turtleFollower_, turtleMaster_, ros::Time(0));
            }
            catch(tf2::TransformException& ex)
            {
                ROS_WARN_STREAM(ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            sendVelCmdToFollower();
            ros::spinOnce();
            loopRate_.sleep();
         }
    }
    
    void TurtleTf2Listener::sendVelCmdToFollower()
    {
        cmdVel_.angular.z = 4.0 * atan2(tf2Stamped_.transform.translation.y, tf2Stamped_.transform.translation.x);
        cmdVel_.linear.x = 0.5 * sqrt(pow(tf2Stamped_.transform.translation.x, 2) + pow(tf2Stamped_.transform.translation.y, 2));
        velPubFollower_.publish(cmdVel_);
        ROS_INFO_STREAM("Vel command send ");
    }
    
    


}
