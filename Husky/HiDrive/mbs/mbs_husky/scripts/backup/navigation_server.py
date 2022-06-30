#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose
from math import pow, atan2, sqrt
from mbs_msgs.msg import GoToPositionAction, GoToPositionFeedback, GoToPositionResult
import actionlib
from tf.transformations import euler_from_quaternion
class HuskyNavigation:

    def __init__(self, action_name):
        self.action_name = action_name
        self.action_server = actionlib.SimpleActionServer(self.action_name, GoToPositionAction, execute_cb=self.goal_cb,
                                                          auto_start=False)

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber("pose", Pose, self.update_pose)
        self.pose = None
        self.result = GoToPositionResult()
        self.feedback = GoToPositionFeedback()
        self.rate = rospy.Rate(10)
        self.action_server.start()
        rospy.loginfo("%s action server started", self.action_name)

    def update_pose(self, data):
        self.pose = data
        # self.pose.position.x = round(self.pose.position.x, 1)
        # self.pose.position.y = round(self.pose.position.y, 1)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.goalX - self.pose.position.x), 2) +
                    pow((goal_pose.goalY - self.pose.position.y), 2))
        # rospy.logwarn("Distance between start: {} and goal: {} is: {}.".format([self.pose.position.x, self.pose.position.y],goal_pose, distance))
        # return distance

    def linear_vel(self, goal_pose, constant=0.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.goalY - self.pose.position.y, goal_pose.goalX - self.pose.position.x)

    def angular_vel(self, goal_pose, constant=0.5):
        euler = euler_from_quaternion([self.pose.orientation.x,
                                       self.pose.orientation.y,
                                       self.pose.orientation.z,
                                       self.pose.orientation.w])
        return constant*(self.steering_angle(goal_pose) - euler[-1])
        # return self.steering_angle(goal_pose)

    def feedback_cb(self, distance_to_goal, theta_to_face):
        self.feedback.linearDistance = distance_to_goal
        self.feedback.angularRotation = theta_to_face
        self.action_server.publish_feedback(self.feedback)        

    def goal_cb(self, goal):
        rospy.loginfo("%s action server called with goal %s"%(self.action_name, goal))
        distance_tolerance = 0.2
        vel_msg = Twist()
        current_linear_distance = self.euclidean_distance(goal) 
        current_angular_distance = self.angular_vel(goal)
        rospy.logwarn("Initial distance between start: {} and goal: {} is: {}.".format([self.pose.position.x, self.pose.position.y], goal, current_linear_distance))
        rospy.logerr("Face angle to goal is: {}".format(current_angular_distance))
        # while current_angular_distance >= 0.1:
        #     rospy.logerr("Face angle to goal is: {}".format(current_angular_distance))
        #     vel_msg.angular.z = 0.5
        #     self.velocity_publisher.publish(vel_msg)
        #     # Publish at the desired rate.
        #     self.rate.sleep()
        #     current_angular_distance = self.angular_vel(goal)

        while current_linear_distance >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal)
            # vel_msg.angular.z = current_angular_distance
            # Publish feedback
            self.feedback_cb(current_linear_distance, current_angular_distance)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()
            self.result.goalLinear = current_linear_distance
            self.result.goalAngular = current_angular_distance
            current_linear_distance = self.euclidean_distance(goal) 
            current_angular_distance = self.angular_vel(goal)
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Setting up the server as succedded 
        rospy.loginfo('%s: Succeeded' % self.action_name)
        self.action_server.set_succeeded(self.result)            

if __name__ == '__main__':
    try:
        action_name = "husky_navigation_server"
        rospy.init_node(action_name, anonymous=False)    
        rospy.loginfo("Started node: {}".format(action_name))    
        x = HuskyNavigation(action_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
