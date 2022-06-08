#include <ros/ros.h>
#include "turtle_tf2_broadcaster/Tft2Broadcaster.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

namespace tf2_broad_caster
{
    Tf2Broadcaster::Tf2Broadcaster(ros::NodeHandle& rnh, const std::string& turtle_name) : 
     nodeHandle_(rnh)
    ,turtle_name_(turtle_name)
     {
        subscribeTopic_ = "turtle1/pose";
        ROS_INFO_STREAM("Subscribe topic "<< subscribeTopic_);
        subscriber_ = nodeHandle_.subscribe(subscribeTopic_, 100, &Tf2Broadcaster::poseCallback, this);
     }
     
     Tf2Broadcaster::~Tf2Broadcaster()
     {
     
     }
     
     void Tf2Broadcaster::broadcast_transform_tf2(float tx, float ty,float tz, float rx, float ry, float rz)
     {
      //  ROS_INFO("TurtleName= %s, x= %.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f ", turtle_name_.c_str(), tx, ty, tz, rx, ry, rz);
        
        geometry_msgs::TransformStamped tf_broadcaster;
        tf_broadcaster.header.stamp = ros::Time::now();
        tf_broadcaster.header.frame_id = "world";
        tf_broadcaster.child_frame_id = turtle_name_;
        
        tf_broadcaster.transform.translation.x = tx;
        tf_broadcaster.transform.translation.y = ty;
        tf_broadcaster.transform.translation.z = tz;
        
        tf2::Quaternion quat;
        quat.setRPY(rx, ry, rz);
        tf_broadcaster.transform.rotation.x = quat.x();
        tf_broadcaster.transform.rotation.y = quat.y();
        tf_broadcaster.transform.rotation.z = quat.z();
        tf_broadcaster.transform.rotation.w = quat.w();
        
        br_.sendTransform(tf_broadcaster);
     }
     
     void Tf2Broadcaster::poseCallback(const turtlesim::PoseConstPtr& msg)
     {
        broadcast_transform_tf2(msg->x, msg->y, 0, 0, 0, msg->theta);
     }
     
     
}
