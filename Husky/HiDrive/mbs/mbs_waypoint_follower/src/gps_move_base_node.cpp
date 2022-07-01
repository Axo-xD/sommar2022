#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
std::string utm_zone;


geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("odom", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("odom", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}

move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next, bool last_point)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    if(last_point == false)
    {
        tf::Matrix3x3 rot_euler;
        tf::Quaternion rot_quat;

        // Calculate quaternion
        float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
        float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
        float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
        float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
        yaw_curr = atan2(delta_y, delta_x);

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
        rot_euler.getRotation(rot_quat);

        goal.target_pose.pose.orientation.x = rot_quat.getX();
        goal.target_pose.pose.orientation.y = rot_quat.getY();
        goal.target_pose.pose.orientation.z = rot_quat.getZ();
        goal.target_pose.pose.orientation.w = rot_quat.getW();
    }
    else
    {
        goal.target_pose.pose.orientation.w = 1.0;
    }

    return goal;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");
    double latitude = 49.90000001835765;
    double longitude = 8.900000008664884; 
    auto utm_point = latLongtoUTM(latitude, longitude);
    std::cout << "UTM X is: " << utm_point.point.x << " whereas UTM Y is: " <<  utm_point.point.y << std::endl;
    // auto map_point = UTMtoMapPoint(utm_point);
    // std::cout << "MAP X is: " << map_point.point.x << " whereas MAP Y is: " <<  map_point.point.y << std::endl;
    // geometry_msgs::PointStamped map_next;
    // auto goal = buildGoal(map_point, map_next, true);
    // std::cout << "Goal X is: " << goal.target_pose.pose.position.x << " whereas Goal Y is: " <<  goal.target_pose.pose.position.y << std::endl;


    // //tell the action client that we want to spin a thread by default
    // MoveBaseClient ac("move_base", true);

    // //wait for the action server to come up
    // while(!ac.waitForServer(ros::Duration(5.0))){
    // ROS_INFO("Waiting for the move_base action server to come up");
    // }
    // ac.sendGoal(goal);  

    // ac.waitForResult();

    // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // ROS_INFO("Hooray, the base moved 1 meter forward");
    // else
    // ROS_INFO("The base failed to move forward 1 meter for some reason");

    return 0;

}