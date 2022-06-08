#include <ros/ros.h>
#include "static_broadcast_tf2/StaticBroadcaster.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>

namespace static_broad_caster
{
    StaticBroadcaster::StaticBroadcaster(ros::NodeHandle& rnh, const std::string& turtle_name) : nodeHandle_(rnh)
    , turtle_name_(turtle_name)
    , static_broadcaster_()
    , pose_x_(0.0), pose_y_(0.0), pose_z_(0.0)
    , pose_rx_(0.0), pose_ry_(0.0), pose_rz_(0.0)
    , use_param_(false)
    {
        use_param_ = readParameters();
    }

    StaticBroadcaster::~StaticBroadcaster()
    {
    
    }
    
     bool StaticBroadcaster::useParameters() const
     {
        return use_param_;
     }
     
    bool StaticBroadcaster::readParameters()
    {
       bool good_param = true;
       good_param &= readParamVal("turtle_name", turtle_name_);
       good_param &= readParamVal("pose_x", pose_x_);
       good_param &= readParamVal("pose_y", pose_y_);
       good_param &= readParamVal("pose_z", pose_z_);
       
       good_param &= readParamVal("pose_rx", pose_rx_);
       good_param &= readParamVal("pose_ry", pose_ry_);
       good_param &= readParamVal("pose_rz", pose_rz_);
       return good_param;
    }
    
     bool StaticBroadcaster::readParamVal(const std::string& param_name, float& val)
     {
       bool read_param = false;
       if(!nodeHandle_.getParam(param_name, val))
        {
            ROS_ERROR_STREAM("Failed to read "<< param_name);
        }
        else
        {
            ROS_INFO_STREAM(param_name << " Val:: "<< val);
            read_param = true;
        }
        return read_param;
     }

     bool StaticBroadcaster::readParamVal(const std::string& param_name, std::string& val)
     {
        bool read_param = false;
        if(!nodeHandle_.getParam(param_name, val))
        {
            ROS_ERROR_STREAM("Failed to read "<< param_name);
        }
        else
        {
            ROS_INFO_STREAM(param_name << " Val:: "<< val);
            read_param = true;
        }
        return read_param;
     }
         
    void StaticBroadcaster::broadcaster_static_tf2(float x_f, float y_f, float z_f, float rx_f, float ry_f, float rz_f)
    {
       geometry_msgs::TransformStamped static_transformedstamped;
       static_transformedstamped.header.stamp = ros::Time::now();
       static_transformedstamped.header.frame_id = "world";
       static_transformedstamped.child_frame_id = turtle_name_;
       static_transformedstamped.transform.translation.x = x_f;
       static_transformedstamped.transform.translation.y = y_f;
       static_transformedstamped.transform.translation.z = z_f;
       
       tf2::Quaternion quat;
       quat.setRPY(rx_f, ry_f, rz_f);
       static_transformedstamped.transform.rotation.x = quat.x();
       static_transformedstamped.transform.rotation.y = quat.y();
       static_transformedstamped.transform.rotation.z = quat.z();
       static_transformedstamped.transform.rotation.w = quat.w();
       static_broadcaster_.sendTransform(static_transformedstamped);
       ROS_INFO("Spinning until killed publishing %s to world", turtle_name_.c_str());  
    }
    
   void StaticBroadcaster::broadcaster_static_tf2(const char*x, const char*y, const char*z, const char *rx, const char* ry, const char* rz)
   {
       ROS_INFO("x= %s, y=%s, z=%s, rx=%s, ry=%s, rz=%s ", x, y, z, rx, ry, rz);
       float x_f = std::stof(x);
       float y_f = std::stof(y);
       float z_f = std::stof(z);
      
       float rx_f = std::stof(rx);
       float ry_f = std::stof(ry);
       float rz_f = std::stof(rz);
       broadcaster_static_tf2(x_f, y_f, z_f, rx_f, ry_f, rz_f);    
   }
    
    void StaticBroadcaster::execute()
    {
        if(use_param_)
        {
            broadcaster_static_tf2(pose_x_, pose_y_, pose_z_, pose_rx_, pose_ry_, pose_rz_);
        }
    }

}
