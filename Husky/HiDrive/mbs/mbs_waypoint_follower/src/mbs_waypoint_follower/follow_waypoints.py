#!/usr/bin/env python

import rospy
from smach import State,StateMachine
import yaml
from geometry_msgs.msg import Pose, PoseArray
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import actionlib
from tf import TransformListener
import math

class navigation_handler(object):
    count = 0
    frame = None
    waypoints = []
    waypoints_file = rospkg.RosPack().get_path('mbs_waypoint_follower') + "/waypoints/waypoints.yaml"

    def __init__(self):
        pass
    
    def convertListToPoseArray(self):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = navigation_handler.frame
        for waypoint in navigation_handler.waypoints:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = waypoint['position']
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = waypoint['orientation']
            pose_array.poses.append(pose)
        return pose_array

class GetPath(State, navigation_handler):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        navigation_handler.__init__(self)
        # Create publsher to publish waypoints as pose array so that one can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

    def initialize_path_queue(self):
        # publish empty waypoint queue as pose array so that one can see them the change in rviz, etc.
        self.poseArray_publisher.publish(self.convertListToPoseArray())

    def execute(self, userdata):
        self.initialize_path_queue()
        with open(navigation_handler.waypoints_file, 'r') as data_stream:
            try:
                data = yaml.safe_load(data_stream)
                navigation_handler.frame = data['frame_id']
                waypoints = data['waypoints']
                for value in waypoints.values():
                    navigation_handler.waypoints.append(value)
            except yaml.YAMLError as exc:
                rospy.logerr(exc)
                return 'failure'
        rospy.logerr("Get Waypoints successfull.")
        rospy.logerr("Get Waypoints successfull.")
        return 'success'

class FollowPath(State, navigation_handler):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        navigation_handler.__init__(self)
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.0)

    def execute(self, userdata):
        # Execute waypoints each in sequence
        waypoints = self.convertListToPoseArray()
        for waypoint in waypoints.poses:
            # Break if preempted
            time.sleep(5)
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = waypoints.header.frame_id
            goal.target_pose.pose.position = waypoint.position
            goal.target_pose.pose.orientation = waypoint.orientation
            rospy.logerr('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.position.x, waypoint.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)
            if not self.distance_tolerance > 0.0:
                self.client.wait_for_result()
                rospy.loginfo("Waiting for %f sec..." % self.duration)
                time.sleep(self.duration)
            else:
                #This is the loop which exist when the robot is near a certain GOAL point.
                distance = 10
                while(distance > self.distance_tolerance):
                    now = rospy.Time.now()
                    self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                    trans, _ = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, now)
                    distance = math.sqrt(pow(waypoint.position.x-trans[0],2)+pow(waypoint.position.y-trans[1],2))
        return 'success'

    def execute1(self, userdata):
        rospy.loginfo(navigation_handler.waypoints)
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FFFFFF GATE #####')
        rospy.loginfo('###############################')
        return 'success'


class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        return 'success'

def main():
    rospy.init_node('mbs_waypoint_follower')
    sm = StateMachine(outcomes=['success', 'over'])
    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success': 'over'})
    outcome = sm.execute()