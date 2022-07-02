#include <ros/ros.h>
#include <urdf/model.h>
#include "simplerobot_description/SimpleRobotDescription.hpp"
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>


namespace simple_robot_descrption
{
    SimpleRobotDescription::SimpleRobotDescription(ros::NodeHandle& rnh, const std::string& urdfFile)
    : nodeHandle_(rnh)
    , urdfFile_(urdfFile)
    , kdlTree_()
    , loopRate_(30)
    , subscriberTopic_("/joint_states")
    , subscriber_()
    {
        if(false == urdfModel_.initFile(urdfFile_)) 
        {
            ROS_ERROR_STREAM("Failed to parse urdf_file");
            throw("Failed to parse urdf_file");
        }
        
        if(false == kdl_parser::treeFromUrdfModel(urdfModel_, kdlTree_))
        {
            ROS_ERROR_STREAM("Failed to construct kdl tree");
            throw("Failed to construct KDL tree");
        }
        jointStates_ = {{"joint1" , 0.0}, {"joint2",  0.0 }, {"joint3",  0.0 }};
        robotPubPtr_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(kdlTree_, urdfModel_);
        subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 100, &SimpleRobotDescription::jointStateCallback, this);
      //  testKDLTreeSerialTraverse();
      //  testKDLTreeRecursiveTraverse(kdlTree_.getRootSegment());
       testKDLSegmentsTraverse();
    }
   
    SimpleRobotDescription::~SimpleRobotDescription()
    {
    
    
    }
    
    void SimpleRobotDescription::broadcasteFrame(float tx, float ty, float tz, float rx, float ry, float rz)
    {
        geometry_msgs::TransformStamped tf_broadcaster;
        tf_broadcaster.header.stamp = ros::Time::now();
        tf_broadcaster.header.frame_id = "world";
        tf_broadcaster.child_frame_id = "base_link";
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
    
    
    void SimpleRobotDescription::execute()
    {
        //robotPubPtr_->publishFixedTransforms(true);
        while(nodeHandle_.ok())
        {
            broadcasteFrame(0, 0, 0, 0, 0, 0);
            robotPubPtr_->publishTransforms(jointStates_, ros::Time::now());
            ros::spinOnce();
            loopRate_.sleep();
        }
        
    }
   
    void SimpleRobotDescription::testKDLTreeSerialTraverse()
    {
        unsigned int numJoints = kdlTree_.getNrOfJoints();
        unsigned int numSegments = kdlTree_.getNrOfSegments();
        ROS_INFO_STREAM("Num of joints "<<numJoints << ", Num of segments "<<numSegments);

        std::vector<KDL::SegmentMap::const_iterator> subSegmentsArray;
        subSegmentsArray.push_back(kdlTree_.getRootSegment());
        while(subSegmentsArray.size() > 0)
        {
            KDL::SegmentMap::const_iterator kdlSegMap = subSegmentsArray.back();
            subSegmentsArray.pop_back();
            ROS_INFO_STREAM("Tree element Name  "<< kdlSegMap->first << ", Index " << (kdlSegMap->second).q_nr); 
            ROS_INFO_STREAM("Segment name "<< (kdlSegMap->second).segment.getName() << ", Index " << (kdlSegMap->second).q_nr << ", Num Children " << (kdlSegMap->second).children.size());
            ROS_INFO_STREAM("-------------------------------------------------------------");  


            if((kdlSegMap->second).children.size() > 0)
            {
                for(auto childElem : (kdlSegMap->second).children)
                {
                    subSegmentsArray.push_back(childElem);
                }
            }   
        }   
    }
   
    void SimpleRobotDescription::testKDLTreeRecursiveTraverse(KDL::SegmentMap::const_iterator rootSegment)
    {
        ROS_INFO_STREAM("Tree element name "<< rootSegment->first << ", Index " << (rootSegment->second).q_nr); 
        ROS_INFO_STREAM("Segment name "<< (rootSegment->second).segment.getName() << ", Index " << (rootSegment->second).q_nr << ", Num Children " << (rootSegment->second).children.size());
        if((rootSegment->second).children.size() > 0)
        {
            for(auto elem : (rootSegment->second).children)
            {
                testKDLTreeRecursiveTraverse(elem);
            }
        }
    }
    
    
     void SimpleRobotDescription::testKDLSegmentsTraverse()
     {
        const KDL::SegmentMap& segMap = kdlTree_.getSegments();
        for(auto elem : segMap)
        {
            ROS_INFO_STREAM("Tree element name "<< elem.first << ", Index " << elem.second.q_nr);
            ROS_INFO_STREAM("Segment name "<< elem.second.segment.getName());
            showSeqmentFrame(elem.second.segment.getFrameToTip());
            if(elem.second.segment.getName() != "base_link")
            {
                ROS_INFO_STREAM("Parent name "<< (elem.second.parent)->first);
            }
            showSeqmentJoint(elem.second.segment.getJoint());
            ROS_INFO_STREAM("-------------------------------------------------------------");
        }
     }
     
     void SimpleRobotDescription::showSeqmentJoint(const KDL::Joint& joint)
     {
        ROS_INFO_STREAM("Joint name " << joint.getName() << ", joint type " << joint.getTypeName()); 
        KDL::Vector jAxis = joint.JointAxis();
        KDL::Vector jOrg  = joint.JointOrigin();
        
        ROS_INFO_STREAM("Joint Axis [" <<jAxis.x() << ", "<< jAxis.y() <<", "<<jAxis.z()<< "], Joint Origin [" <<jOrg.x() << ", "<< jOrg.y() <<", "<<jOrg.z()<< "]"); 
     }
     
     void SimpleRobotDescription::showSeqmentFrame(const KDL::Frame& frame)
     {
        KDL::Rotation rotM = frame.M;
        KDL::Vector pos = frame.p;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        rotM.GetRPY(roll, pitch, yaw);
        ROS_INFO_STREAM("RPY [" <<roll << ", "<< pitch <<", "<<yaw<< "], XYZ [" <<pos.x() << ", "<< pos.y() <<", "<<pos.z()<< "]"); 
     }
     
     void SimpleRobotDescription::showURDFModel()
     {
     
     }
     
     void SimpleRobotDescription::jointStateCallback(const sensor_msgs::JointState& jointMsg)
     {
        int numJoints = jointMsg.name.size();
        
        ROS_INFO_STREAM("Num joints" << numJoints); 
        for(int i = 0; i < numJoints; i++)
        {
            ROS_INFO_STREAM("Joint index:: " << i << "name:: " << jointMsg.name[i] << "posiiton "<< jointMsg.position[i]); 
            jointStates_[jointMsg.name[i]] = jointMsg.position[i];
        }
     }
}
