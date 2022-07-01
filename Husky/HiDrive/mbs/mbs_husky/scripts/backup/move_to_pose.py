#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from random import random
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
# simulation parameters
Kp_rho = 9
Kp_alpha = 15
Kp_beta = -3
dt = 0.01

# Robot specifications
MAX_LINEAR_SPEED = 50
MAX_ANGULAR_SPEED = 25

show_animation = True


class MoveToPose(object):
    def __init__(self):
        self.pose = None

    def poseUpdate(self, data):
        self.pose = data

    def trajPlanner(self, start, goal):
        """
        rho is the distance between the robot and the goal position
        alpha is the angle to the goal relative to the heading of the robot
        beta is the angle between the robot's position and the goal position plus the goal angle

        Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
        Kp_beta*beta rotates the line so that it is parallel to the goal angle
        """
        x , y , theta = start
        x_goal , y_goal , theta_goal = goal


        x_diff = x_goal - x
        y_diff = y_goal - y

        x_traj, y_traj = [], []

        rho = np.hypot(x_diff, y_diff)

        alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        print(alpha, beta)
        while rho > 0.001 or abs(alpha) > 0.001:
            x_traj.append(x)
            y_traj.append(y)

            x_diff = x_goal - x
            y_diff = y_goal - y
            print(beta)

            # Restrict alpha and beta (angle differences) to the range
            # [-pi, pi] to prevent unstable behavior e.g. difference going
            # from 0 rad to 2*pi rad with slight turn

            rho = np.hypot(x_diff, y_diff)
            alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi)
            beta = (theta_goal - theta - alpha + np.pi)
            # beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

            v = Kp_rho * rho
            w = Kp_alpha * alpha + Kp_beta * beta

            if alpha > np.pi / 2 or alpha < -np.pi / 2:
                v = -v

            if abs(v) > MAX_LINEAR_SPEED:
                v = np.sign(v) * MAX_LINEAR_SPEED

            if abs(w) > MAX_ANGULAR_SPEED:
                w = np.sign(w) * MAX_ANGULAR_SPEED

            theta = theta + w * dt
            x = x + v * np.cos(theta) * dt
            y = y + v * np.sin(theta) * dt
            vel = Twist()
            vel.linear.x = v 
            vel.angular.z = w
            try:
                # x = self.pose.position.x
                # y = self.pose.position.y
                # self.cmd_pub.publish(vel)
                # print(rho)
                pass
            except AttributeError:
                pass
            
            print(rho)
            if show_animation:  # pragma: no cover
                plt.cla()
                plt.arrow(start[0], start[1], np.cos(start[-1]),
                        np.sin(start[-1]), color='r', width=0.1)
                plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                        np.sin(theta_goal), color='g', width=0.1)
                self.plot_vehicle(x, y, theta, x_traj, y_traj)

    def plot_vehicle(self, x, y, theta, x_traj, y_traj):  # pragma: no cover
        p1_i = np.array([0.5, 0, 1]).T
        p2_i = np.array([-0.5, 0.25, 1]).T
        p3_i = np.array([-0.5, -0.25, 1]).T

        T = self.transformation_matrix(x, y, theta)
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)

        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
        plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

        plt.plot(x_traj, y_traj, 'b--')

        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

        plt.xlim(int(self.start[0]) - 10, int(self.goal[0]) + 10)
        plt.ylim(int(self.start[1]) - 10, int(self.goal[1]) + 10)

        plt.pause(dt)

    def transformation_matrix(self, x, y, theta):
        return np.array([
                        [np.cos(theta), -np.sin(theta), x],
                        [np.sin(theta), np.cos(theta), y],
                        [0, 0, 1]
                        ])
  
    def planner(self):
        import rospy
        from tf.transformations import euler_from_quaternion

        rospy.init_node('move_to_base')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber("pose", Pose, self.poseUpdate)

        while self.pose == None and not rospy.is_shutdown():
            pass
        x = self.pose.position.x 
        y = self.pose.position.y
        x_goal = x + 1.0
        y_goal = y + 1.0
        x_diff = x_goal - x
        y_diff = y_goal - y

        theta = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])[-1]
        theta_goal = 0.0

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
            
        try:
            # x = self.pose.position.x
            # y = self.pose.position.y
            # self.cmd_pub.publish(vel)
            print("x: {}, y: {},".format(x, y))
            print("x_goal: {}, y_goal: {}.".format(x_goal, y_goal))
            print("x_diff: {}, y_diff: {}".format(x_diff, y_diff))
            print(rho, alpha)
        except AttributeError:
            pass

    def main(self):
        self.start = [5647403.93303, 331549.39349, -3.1415]
        self.goal  = [5647403.93303, 331549.39349, 4.7123]
        print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
            (self.start[0], self.start[1], self.start[-1]))
        print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
            (self.goal[0], self.goal[1], self.goal[-1]))
        self.trajPlanner(self.start, self.goal)
        # self.planner()

if __name__ == '__main__':
    trajPlanner = MoveToPose()
    trajPlanner.main()