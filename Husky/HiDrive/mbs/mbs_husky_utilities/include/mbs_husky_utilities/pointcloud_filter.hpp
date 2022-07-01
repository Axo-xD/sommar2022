#ifndef POINTCLOUD_FILTER_H
#define POINTCLOUD_FILTER_H 

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <cmath>

class PointCloudFilter {
    private:
    ros::Subscriber pcl_sub;
    ros::Publisher  pcl_pub;    
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    double angle_min, angle_max, c_radius;
    // PointCloud::Ptr filtered_pcl;

    public:
    PointCloudFilter(ros::NodeHandle *nh, ros::NodeHandle *p_nh);
    void pclCallback(const PointCloud::ConstPtr& msg);
    float distanceFromOrigin(float x, float y, float z);
};
#endif 