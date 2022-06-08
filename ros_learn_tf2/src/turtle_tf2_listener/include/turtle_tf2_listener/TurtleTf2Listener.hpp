#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Spawn.h>

namespace turtle_tf2_listener
{
    class TurtleTf2Listener
    {
        public:
            TurtleTf2Listener(ros::NodeHandle& rnh);
            virtual ~TurtleTf2Listener();
            void execute();
            void sendVelCmdToFollower();
            void spawnTurtle();
        private:
            ros::NodeHandle& nodeHandle_;
            geometry_msgs::Twist cmdVel_;
            tf2_ros::Buffer tf2Buffer_;
            tf2_ros::TransformListener tf2Listener_;
            geometry_msgs::TransformStamped tf2Stamped_;
            std::string turtleMaster_;
            std::string turtleFollower_;
            ros::Publisher velPubFollower_;
            ros::Rate loopRate_;
            turtlesim::Spawn turtle1_;
            ros::ServiceClient spawner_;
    };

}
