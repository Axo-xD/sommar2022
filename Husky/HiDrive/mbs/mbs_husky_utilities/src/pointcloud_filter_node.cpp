#include "mbs_husky_utilities/pointcloud_filter.hpp"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "pcl_filter_node");
    ros::NodeHandle nh, p_nh("~");   
    PointCloudFilter filter = PointCloudFilter(&nh, &p_nh);
    ros::Rate loop_rate(4);    
    while (nh.ok()){
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}