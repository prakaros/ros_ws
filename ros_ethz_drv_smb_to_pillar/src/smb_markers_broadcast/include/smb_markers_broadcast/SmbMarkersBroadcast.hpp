#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

namespace smb_markers_br
{
    class SmbMarkersBroadcast
    {
         public:
            SmbMarkersBroadcast(ros::NodeHandle& rnh);
            virtual ~SmbMarkersBroadcast();
            void execute();
            void sendVizMarker();
         private:
            ros::NodeHandle& nodeHandle_;
            geometry_msgs::TransformStamped tfStamped_;
            visualization_msgs::Marker markerMsg_;
            ros::Rate loop_rate_;
            std::string targetFrame_;
            std::string sourceFrame_;
            ros::Publisher vizPublisher_;
            tf2_ros::Buffer tfBuffer_; 
            tf2_ros::TransformListener tfListener_;
            int num_message_;
 
    };


}
