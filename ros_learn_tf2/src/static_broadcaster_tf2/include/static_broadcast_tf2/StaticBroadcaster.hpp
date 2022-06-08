#pragma once
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace static_broad_caster
{
    class StaticBroadcaster
    {
        public:
            StaticBroadcaster(ros::NodeHandle & rnh, const std::string& turtle_name);
            virtual ~StaticBroadcaster();
            void broadcaster_static_tf2(float x, float y, float z, float rx, float ry, float rz);
            void broadcaster_static_tf2(const char* x, const char* y, const char* z, const char* rx, const char* ry, const char*rz);
            bool useParameters() const;
            void execute();
        private:
         	    bool readParameters();
         	    bool readParamVal(const std::string& param_name, float& val);
         	    bool readParamVal(const std::string& param_name, std::string& val);
        private:
            ros::NodeHandle& nodeHandle_;
            std::string turtle_name_;
            tf2_ros::StaticTransformBroadcaster static_broadcaster_;
            float pose_x_, pose_y_, pose_z_;
            float pose_rx_, pose_ry_, pose_rz_;
            bool use_param_;
    };

}
