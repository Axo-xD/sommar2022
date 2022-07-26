import rospy
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion
from numpy import arctan2, tanh, array
from numpy.linalg import norm
from time import sleep
class Controller(object):
    def __init__(self):
        self.angular_goal_tolerance = 0.05
        self.linear_goal_tolerance = 0.15

        self.is_utm_x_alligned = -1.0
        self.is_utm_y_alligned = 1.0

        self.pose_flag = False
        self.goal_active = False

        self.current_goal = None
        self.current_goal_distance = None
        self.robot_heading = None
        self.goal_angle = None

        self.pose = Pose()
        rospy.init_node("gps_navigation_node")
        rospy.loginfo("GPS Navigation node initialized.")
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/pose', Pose, self.poseCallback)

        self.limit_velocity        = lambda velocity, range=1.0: tanh(velocity)*range
        self.get_distance_to_goal  = lambda : norm(array([-self.current_goal.position.x, self.current_goal.position.y]) - array([self.pose.position.x, self.pose.position.y]))
        self.get_angle_to_goal     = lambda : arctan2(self.current_goal.position.y - self.pose.position.y, self.current_goal.position.x - self.pose.position.x)                
        self.get_robot_heading     = lambda pose_msg: euler_from_quaternion([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])[-1]

    def run(self):
        goal_point = [[5647407.384872, 331556.408005], [5647398.719889, 331550.547161]]
        for point in goal_point:
            self.current_goal  = self.createTestGoal(*point)
            self.allignRobot()
            self.moveToGoal()
            rospy.loginfo("Reached waypoint")
            sleep(5)

    def poseCallback(self, pose_msg): 
        self.pose.position.x  = pose_msg.position.x*self.is_utm_x_alligned
        self.pose.position.y  = pose_msg.position.y*self.is_utm_y_alligned
        self.pose.orientation = pose_msg.orientation
        self.pose_flag = True
        if self.goal_active:
            self.robot_heading = self.get_robot_heading(pose_msg)
            self.goal_angle    = self.get_angle_to_goal()
            self.current_goal_distance = self.get_distance_to_goal()
    
    def allignRobot(self):
        while not self.goal_active:
            pass
        rospy.loginfo("robot allign request received.")
        cmd_vel = Twist()
        alpha = self.goal_angle - self.robot_heading 
        while abs(alpha) > self.angular_goal_tolerance and not rospy.is_shutdown():
            alpha = self.goal_angle - self.robot_heading 
            cmd_vel.angular.z = self.limit_velocity(alpha, 0.5)
            self.cmd_pub.publish(cmd_vel)
            rospy.loginfo_throttle(5, "Face angle to goal is {}.".format(alpha))

    def moveToGoal(self):
        cmd_vel = Twist()
        rospy.loginfo("Start point: {}, goal_point: {} and the distance is: {}.".format([self.pose.position.x, self.pose.position.y], [self.current_goal.position.x, self.current_goal.position.y], self.current_goal_distance))
        while self.current_goal_distance > self.linear_goal_tolerance and not rospy.is_shutdown():
            alpha = self.goal_angle - self.robot_heading 
            cmd_vel.linear.x = self.limit_velocity(self.current_goal_distance, 0.75)
            cmd_vel.angular.z = self.limit_velocity(alpha)
            self.cmd_pub.publish(cmd_vel)
            rospy.loginfo_throttle(5, "Distance between current {} and goal point {} is {}.".format([self.pose.position.x, self.pose.position.y],[self.current_goal.position.x, self.current_goal.position.y] , self.current_goal_distance ))
        self.goal_active = False

    def createTestGoal(self, x=0.0, y=0.0):
        self.goal_active = True
        while not self.pose_flag and not rospy.is_shutdown(): # Wait until pose message is received.
            pass
        goal = Pose()
        # goal.position.x = self.pose.position.x + dx
        # goal.position.y = self.pose.position.y + dy
        goal.position.x = x
        goal.position.y = y
        return goal

if __name__ == '__main__':
    controller = Controller()
    controller.run()