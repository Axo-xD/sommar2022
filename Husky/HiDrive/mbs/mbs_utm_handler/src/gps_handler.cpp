#include <ros/ros.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include "mbs_msgs/GPStoMAP.h"

geometry_msgs::PointStamped latLontoUTM(double lat, double lon) {
    double utm_x = 0, utm_y = 0;
    std::string utm_zone;
    geometry_msgs::PointStamped utm;

    RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone);

    utm.header.frame_id = "utm";
    utm.header.stamp = ros::Time(0);
    utm.point.x = utm_x;
    utm.point.y = utm_y;
    utm.point.z = 0;

    return utm;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped utm) {
    tf::TransformListener listener; //create transformlistener object called listener
    geometry_msgs::PointStamped map;
    bool notDone = true;
    ros::Time time_now = ros::Time::now();
    while(notDone) {
        try {
            map.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("odom", utm, map);
            notDone = false;
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map;
}

bool gpsHandler(mbs_msgs::GPStoMAP::Request  &request, mbs_msgs::GPStoMAP::Response &response) {
    geometry_msgs::PointStamped utm, map;
    response.utm = latLontoUTM(request.gps.latitude, request.gps.longitude);
    response.utm.point.x = response.utm.point.x + request.dx;
    response.utm.point.y = response.utm.point.y + request.dy; 
    response.map = UTMtoMapPoint(response.utm);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_to_odom_node"); //initiate node called gps_waypoint 
    ros::NodeHandle n;
    double lat_goal, lon_goal;
    ROS_INFO("Initiated gps_to_odom node");
    ros::ServiceServer service = n.advertiseService("gps_to_map", gpsHandler);
    ros::spin();
    return 0;
}