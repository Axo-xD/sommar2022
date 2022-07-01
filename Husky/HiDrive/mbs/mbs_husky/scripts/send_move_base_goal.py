#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from mbs_msgs.srv import GPStoMAP, GPStoMAPRequest
from sensor_msgs.msg import NavSatFix

def create_move_base_goal(x, y):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quaternion = quaternion_from_euler(0, 0, 0)
    goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = [0, 0, 0.5, 0.5]    
    return goal

def gps_to_move_base_goal(dx = 0.0, dy = 0.0):
    # gps = rospy.wait_for_message("mbs_husky/gps/filtered", NavSatFix)       
    gps_msg = GPStoMAPRequest()
    # gps_msg.gps.latitude = gps.latitude
    # gps_msg.gps.longitude = gps.longitude
    # gps_msg.dx = dx
    # gps_msg.dy = dy
    gps_msg.gps.latitude  = 50.9535875206
    gps_msg.gps.longitude = 6.60168353129 
    gps_to_map_server = rospy.ServiceProxy('gps_to_map', GPStoMAP)
    gps_res = gps_to_map_server(gps_msg)

    rospy.loginfo("".format())
    rospy.logwarn("(lat, lon) -> utm(x, y): {}, {}".format(gps_res.utm.point.x, gps_res.utm.point.y))
    rospy.logwarn("(lat, lon) -> map(x, y): {}, {}".format(gps_res.map.point.x, gps_res.map.point.y))
    rospy.loginfo("".format())
    goal = create_move_base_goal(gps_res.map.point.x, gps_res.map.point.y)
    rospy.loginfo(goal)
    return goal

def movebase_client():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server.")
    client.wait_for_server()
    goal = gps_to_move_base_goal(4.0)
    rospy.loginfo("Sending goal to the move_base action server.")
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal accomplished.")
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        rospy.loginfo("Move base client started!")
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("Navigation test finished.")
