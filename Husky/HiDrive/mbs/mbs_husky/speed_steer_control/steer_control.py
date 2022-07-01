"""

Path tracking simulation with LQR steering control and PID speed control.

author Atsushi Sakai (@Atsushi_twi)

"""
import scipy.linalg as la
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
# import rospy
from geometry_msgs.msg import Twist, Pose
sys.path.append("..")

try:
    import cubic_spline_planner
    from utils import Utils, State, LqrController
except:
    raise


def PIDControl(target, current, Kp=1.0):
    a = Kp * (target - current)

    return a

class Controller(object):
    def __init__(self):
        # rospy.init_node('cmd_pub')
        self.utils = Utils()
        self.controller = LqrController()
        # self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.state = None

    def lqr_steering_control(self, state, cx, cy, cyaw, ck, pe, pth_e):
        ind, e = self.utils.calc_nearest_index(state, cx, cy, cyaw)

        k = ck[ind]
        v = state.v
        th_e = self.utils.pi_2_pi(state.yaw - cyaw[ind])

        A = np.zeros((4, 4))
        A[0, 0] = 1.0
        A[0, 1] = self.utils.dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = self.utils.dt
        # print(A)

        B = np.zeros((4, 1))
        B[3, 0] = v / self.utils.L

        K, _, _ = self.utils.dlqr(A, B, self.utils.Q, self.utils.R)

        x = np.zeros((4, 1))

        x[0, 0] = e
        x[1, 0] = (e - pe) / self.utils.dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - pth_e) / self.utils.dt

        ff = math.atan2(self.utils.L * k, 1)
        fb = self.utils.pi_2_pi((np.dot(-K, x))[0, 0])

        delta = ff + fb

        return delta, ind, e, th_e


    def closed_loop_prediction(self, cx, cy, cyaw, ck, speed_profile, goal):
        T = 500.0  # max simulation time
        goal_dis = 0.3
        stop_speed = 0.05

        state = State(x=-0.0, y=-0.0, yaw=3.1415, v=0.0)

        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]

        e, e_th = 0.0, 0.0

        while T >= time:
            dl, target_ind, e, e_th = self.lqr_steering_control(
                state, cx, cy, cyaw, ck, e, e_th)

            ai = PIDControl(speed_profile[target_ind], state.v)
            state = self.utils.update(state, ai, dl)

            if abs(state.v) <= stop_speed:
                target_ind += 1

            time = time + self.utils.dt

            # check goal
            dx = state.x - goal[0]
            dy = state.y - goal[1]
            if math.hypot(dx, dy) <= goal_dis:
                print("Goal")
                break

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)

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
                plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                        + ",target index:" + str(target_ind))
                plt.pause(0.0001)

        return t, x, y, yaw, v

    def main(self):
        print("LQR steering control tracking start!!")
        ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
        ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
        goal = [ax[-1], ay[-1]]

        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=0.1)
        target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s

        sp = self.utils.calc_speed_steer_profile(cx, cy, cyaw, target_speed)

        t, x, y, yaw, v = self.closed_loop_prediction(cx, cy, cyaw, ck, sp, goal)

        show_animation = False
        if show_animation:  # pragma: no cover
            plt.close()
            plt.subplots(1)
            plt.plot(ax, ay, "xb", label="input")
            plt.plot(cx, cy, "-r", label="spline")
            plt.plot(x, y, "-g", label="tracking")
            plt.grid(True)
            plt.axis("equal")
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.legend()

            plt.subplots(1)
            plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="yaw")
            plt.grid(True)
            plt.legend()
            plt.xlabel("line length[m]")
            plt.ylabel("yaw angle[deg]")

            plt.subplots(1)
            plt.plot(s, ck, "-r", label="curvature")
            plt.grid(True)
            plt.legend()
            plt.xlabel("line length[m]")
            plt.ylabel("curvature [1/m]")

            plt.show()


if __name__ == '__main__':
    controller = Controller()
    controller.main()

