#include "mbs_husky_utilities/pointcloud_filter.hpp"
#include <iostream>
PointCloudFilter::PointCloudFilter(ros::NodeHandle *nh, ros::NodeHandle *p_nh) {
    pcl_sub = nh->subscribe("points", 1, &PointCloudFilter::pclCallback, this);
    pcl_pub = nh->advertise<PointCloud>("points/filtered", 1);
    p_nh->getParam("angle_min", angle_min);
    p_nh->getParam("angle_max", angle_max);
    p_nh->getParam("c_radius", c_radius);
}

float PointCloudFilter::distanceFromOrigin(float x, float y, float z) {
    return sqrt(pow(x - 0, 2) + pow(y - 0, 2) + pow(z - 0, 2));
}

void PointCloudFilter::pclCallback(const PointCloud::ConstPtr& msg) {
    PointCloud::Ptr filtered_pcl (new PointCloud);
    filtered_pcl->header.frame_id = "os_sensor";
    // filtered_pcl->width = 2048;
    // filtered_pcl->height = 32;   

    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points) {
        if (angle_min <= atan2(pt.y, pt.x) && atan2(pt.y, pt.x) <= angle_max) {    
            if (distanceFromOrigin(pt.x, pt.y, pt.z) > c_radius) {
                pcl_conversions::toPCL(ros::Time::now(), filtered_pcl->header.stamp);            
                filtered_pcl->points.push_back(pt);
            }                    
        }  
    }
    pcl_pub.publish(filtered_pcl);
}