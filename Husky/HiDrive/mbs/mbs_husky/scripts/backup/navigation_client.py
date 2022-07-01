#!/usr/bin/env python

import rospy
import actionlib
from mbs_msgs.msg import GoToPositionAction, GoToPositionGoal
import geonav_transform.geonav_conversions as gc
reload(gc)

class HuskyNavigationClient:

    def __init__(self):
        self.client = actionlib.SimpleActionClient('husky_navigation_server', GoToPositionAction)
        # Waits for the action server to start up
        rospy.loginfo("Waiting for husky_navigation_server to come up...")
        self.client.wait_for_server()
        rospy.loginfo("husky_navigation_server online...")
        self.run()

    def run(self):
        goals = [(50.9536075349, 6.6017098901)]
        goal = GoToPositionGoal()
        for point in goals:
            goal.goalX, goal.goalY, _ = gc.LLtoUTM(*point)
            goal.goalX, goal.goalY = [5647397.75847, 331551.025987]
            rospy.loginfo("Sending goal to husky_navigation_server with coordinates (%s %s): ", goal.goalX, goal.goalY)
            self.client.send_goal(goal, done_cb=self.result_callback, active_cb=None, feedback_cb=self.feedback_callback)
            self.client.wait_for_result()

    def result_callback(self, state, result):
        rospy.loginfo("Result callback status, angular position: %s, linear position: %s", result.goalAngular, result.goalLinear)

    def feedback_callback(self, feedback):
        rospy.loginfo("Rotation remaining: %s, Linear distance remaining: %s", feedback.angularRotation, feedback.linearDistance)

if __name__ == "__main__":
    try:
        rospy.init_node("husky_navigation_client", anonymous=True)
        ac = HuskyNavigationClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("%s interrupt exception", rospy.get_name())