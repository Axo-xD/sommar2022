"""

Path tracking simulation with LQR speed and steering control

author Atsushi Sakai (@Atsushi_twi)

"""
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist, Pose
sys.path.append("..")

try:
    import cubic_spline_planner
    from utils import Utils, State, LqrController
except ImportError:
    raise

# ax = [5527517.136300883, 5527520.956959956]
# ay = [492818.4390261867, 492818.4420334147]

ax = []
ay = []

class SpeedSteerControl(object):
    def __init__(self):
        rospy.init_node('cmd_pub')
        self.utils = Utils()
        self.controller = LqrController()
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.state = None
    #     rospy.Subscriber('pose', Pose, self.poseCallback,)
    #     self.current_pose = None

    # def poseCallback(self, current_pose):
    #     rospy.logerr("Pose received")
    #     self.current_pose = current_pose

    def do_simulation(self, cx, cy, cyaw, ck, speed_profile, goal):
        global ax, ay
        T = 500.0  # max simulation time
        goal_dis = 0.3
        stop_speed = 0.05

        time = 0.0
        x = [self.state.x]
        y = [self.state.y]
        yaw = [self.state.yaw]
        v = [self.state.v]
        t = [0.0]

        e, e_th = 0.0, 0.0

        while T >= time:
            dl, target_ind, e, e_th, ai = self.controller.lqr_speed_steering_control(self.state, cx, cy, cyaw, ck, e, e_th, speed_profile)

            self.state = self.utils.update(self.state, ai, dl)

            if abs(self.state.v) <= stop_speed:
                target_ind += 1

            time = time + self.utils.dt

            # check goal
            dx = self.state.x - goal[0]
            dy = self.state.y - goal[1]
            if math.hypot(dx, dy) <= goal_dis:
                print("Goal")
                break
            x.append(self.state.x)
            y.append(self.state.y)
            yaw.append(self.state.yaw)
            v.append(self.state.v)
            t.append(time)
            cmd = Twist()
            cmd.linear.x = self.state.v
            cmd.angular.z = self.state.yaw
            self.cmd_pub.publish(cmd)
            if target_ind % 1 == 0 and self.utils.show_animation:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(x, y, "ob", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("speed[km/h]:" + str(round(self.state.v * 3.6, 2))
                        + ",target index:" + str(target_ind))
                plt.pause(0.0001)

        return t, x, y, yaw, v

    def main(self, ):
        global ax, ay
        print("LQR steering control tracking start!!")
        # g_x = 5527524.956959956
        # g_y = 492818.4389866238
        current_pose = rospy.wait_for_message('pose', Pose)
        dx = 4
        g_x = current_pose.position.x + dx
        g_y = current_pose.position.y
        self.state = State()
        rospy.logerr("State message: {}, {}, {}".format(self.state.x, self.state.y, self.state.yaw))        
        ax = [self.state.x, g_x]
        ay = [self.state.y, g_y]
        goal = [ax[-1], ay[-1]]

        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=0.1)

        target_speed = 2

        sp = self.utils.calc_speed_profile(cyaw, target_speed)
        t, x, y, yaw, v = self.do_simulation(cx, cy, cyaw, ck, sp, goal)


if __name__ == '__main__':
    controller = SpeedSteerControl()
    controller.main()
