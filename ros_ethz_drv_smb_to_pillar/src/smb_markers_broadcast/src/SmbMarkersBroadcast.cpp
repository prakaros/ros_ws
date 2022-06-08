#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include "smb_markers_broadcast/SmbMarkersBroadcast.hpp"

namespace smb_markers_br
{
    SmbMarkersBroadcast::SmbMarkersBroadcast(ros::NodeHandle& rnh) :
    nodeHandle_(rnh)
    ,tfStamped_()
    ,markerMsg_()
    ,loop_rate_(10)  //This is hz
    ,targetFrame_("odom")
    ,sourceFrame_("rslidar")
    ,vizPublisher_()
    ,tfBuffer_() 
    ,tfListener_(tfBuffer_)
    , num_message_(0)
    {
        vizPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    }
    
    SmbMarkersBroadcast::~SmbMarkersBroadcast()
    {
    
    }
    
    void SmbMarkersBroadcast::execute()
    {
        
        while(nodeHandle_.ok())
        {
            try
            {
                tfStamped_ = tfBuffer_.lookupTransform(targetFrame_, sourceFrame_, ros::Time(0), ros::Duration(3.0));
            }
            catch(tf2::TransformException& ex)
            {
                ROS_WARN_STREAM(ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            sendVizMarker();
        }
    }
    
    void SmbMarkersBroadcast::sendVizMarker()
    {
        markerMsg_.header.frame_id = "odom";
        markerMsg_.header.stamp = ros::Time();
        markerMsg_.ns = "smb_markers_broadcast";
        markerMsg_.id = num_message_++;
        markerMsg_.type = visualization_msgs::Marker::SPHERE;
        markerMsg_.action = visualization_msgs::Marker::ADD;
        markerMsg_.pose.position.x = tfStamped_.transform.translation.x;
        markerMsg_.pose.position.y = tfStamped_.transform.translation.y;
        markerMsg_.pose.position.z = tfStamped_.transform.translation.z;
        markerMsg_.pose.orientation.x = tfStamped_.transform.rotation.x;
        markerMsg_.pose.orientation.y = tfStamped_.transform.rotation.y;
        markerMsg_.pose.orientation.z = tfStamped_.transform.rotation.z;
        markerMsg_.pose.orientation.w = tfStamped_.transform.rotation.w;
        markerMsg_.scale.x = 0.1;
        markerMsg_.scale.y = 0.1;
        markerMsg_.scale.z = 0.1;
        markerMsg_.color.a = 1.0; // Don't forget to set the alpha!
        markerMsg_.color.r = 0.0;
        markerMsg_.color.g = 1.0;
        markerMsg_.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
       // markerMsg_.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        vizPublisher_.publish( markerMsg_ );
    }

}
