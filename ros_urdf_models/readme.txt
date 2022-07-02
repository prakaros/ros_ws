1. For base_link parent gives segmentation fault with kdl library,
 For example code below
     void SimpleRobotDescription::testKDLSegmentsTraverse()
     {
        const KDL::SegmentMap& segMap = kdlTree_.getSegments();
        for(auto elem : segMap)
        {
            ROS_INFO_STREAM("Tree element name "<< elem.first << ", Index " << elem.second.q_nr);
            ROS_INFO_STREAM("Segment name "<< elem.second.segment.getName());
            if(elem.second.segment.getName() != "base_link")
            {
                ROS_INFO_STREAM("Parent name "<< (elem.second.parent)->first);
            }
            showJoint(elem.second.segment.getJoint());
            ROS_INFO_STREAM("-------------------------------------------------------------");
        }
     }
   
Here if segment name is base_link i.e root then we there segmentation fault while accesing the parent first.

2. The fixed joint in URDF is converted to Joint::None in KDL. And KDL Joint::None is not counted in number joints.
No. of joints in KDL::Tree == Total no. of joints in URDF - No. of fixed joints in URDF.

3. Always check urdf file with urdf checker. In rviz if something is incorrect in urdf files then all frames are shown at one place.
check_urdf <urdf_file_name>
urdf_to_graphviz <urdf_file_name> //This will create a pdf file for links and joints visualization.
evince urdf_file_name.pdf

Use xacro dsl to properly convert urdf file, even if it is already in urdf format. This is specially needed for joint state publisher.
<param name="robot_description" command="$(find xacro)/xacro $(arg urdf_file)" />

