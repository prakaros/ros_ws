#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include "simplerobot_description/SimpleRobotDescription.hpp"

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "robot_description");
    ROS_INFO_STREAM("Number of arguments "<<argc);
    
    ROS_INFO_STREAM("Argv 0 " << argv[0]);
    
    std::string urdf_file = argv[1];
    ROS_INFO_STREAM("Urdf filename " << urdf_file);
    ros::NodeHandle nodeHandle("~");
    simple_robot_descrption::SimpleRobotDescription simpleRobDescr_(nodeHandle, urdf_file);
    
    simpleRobDescr_.execute();
 #if 0   
    urdf::Model model_;
    KDL::Tree my_tree_;
    
    if(false == model_.initFile(urdf_file)) {
        ROS_ERROR_STREAM("Failed to parse urdf_file");
        return -1;
    }
    
    ROS_INFO_STREAM("Successfully parse urdf file");
    
    if(false == kdl_parser::treeFromUrdfModel(model_, my_tree_))
    {
        ROS_ERROR_STREAM("Failed to construct kdl tree");
        return -1;
    }
#endif
    return 0;
}
