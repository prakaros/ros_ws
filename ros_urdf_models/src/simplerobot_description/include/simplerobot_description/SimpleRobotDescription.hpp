#pragma once
#include <ros/ros.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

namespace simple_robot_descrption
{
    class SimpleRobotDescription
    {
        public:
            SimpleRobotDescription(ros::NodeHandle& rnh, const std::string& urdfFile);
            ~SimpleRobotDescription();
            void execute();
        private:
            void broadcasteFrame(float tx, float ty, float tz, float rx, float ry, float rz);
            void testKDLTreeSerialTraverse();
            void testKDLTreeRecursiveTraverse(KDL::SegmentMap::const_iterator rootSegment);
            void testKDLSegmentsTraverse();
            void showSeqmentJoint(const KDL::Joint& joint);
            void showSeqmentFrame(const KDL::Frame& frame);
            void showURDFModel();
            void jointStateCallback(const sensor_msgs::JointState& jointMsg);
        private:
            ros::NodeHandle& nodeHandle_;
            const std::string& urdfFile_;
            urdf::Model urdfModel_;
            KDL::Tree kdlTree_;
            robot_state_publisher::RobotStatePublisher robotPub_;
            ros::Rate loopRate_;
            std::shared_ptr<robot_state_publisher::RobotStatePublisher> robotPubPtr_;
            std::map<std::string, double> jointStates_;
            tf2_ros::TransformBroadcaster br_;
            std::string subscriberTopic_;
            ros::Subscriber subscriber_;
                 
    };
}
