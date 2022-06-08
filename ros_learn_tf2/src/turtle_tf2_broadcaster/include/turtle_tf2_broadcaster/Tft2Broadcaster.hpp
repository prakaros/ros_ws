#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/Pose.h>

namespace tf2_broad_caster
{
    class Tf2Broadcaster
    {
        public:
            Tf2Broadcaster(ros::NodeHandle& rnh, const std::string& turtle_name);
            virtual ~Tf2Broadcaster();
            void broadcast_transform_tf2(float x, float y,float z, float rx, float ry,
             float rz);
        private:
        	void poseCallback(const turtlesim::PoseConstPtr& msg);
        private:
            ros::NodeHandle& nodeHandle_;
            std::string turtle_name_;
            std::string subscribeTopic_;
            ros::Subscriber subscriber_;
            tf2_ros::TransformBroadcaster br_;
    };
}
