import numpy as np
import scipy.linalg as la
import math
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
class State:
    # def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        # self.x = x
        # self.y = y
        # self.yaw = yaw
        # self.v = v

    def __init__(self, v=0.0):
        rospy.loginfo('Pose message does not received waiting ...')
        current_pose = rospy.wait_for_message('pose', Pose)
        rospy.loginfo('Pose message recieved.')
        self.x = current_pose.position.x
        self.y = current_pose.position.y
        robot_yaw = euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])[-1]
        self.yaw = robot_yaw
        self.v = v

class Utils(object):
    def __init__(self):
        # LQR parameter
        self.lqr_Q = np.eye(5)
        self.lqr_R = np.eye(2)
        self.dt = 0.1  # time tick[s]
        self.L = 0.5  # Wheel base of the vehicle [m]
        self.max_steer = np.deg2rad(45.0)  # maximum steering angle[rad]
        self.Q = np.eye(4)
        self.R = np.eye(1)
        self.show_animation = True   
        rospy.Subscriber('pose', Pose, self.poseCallback,)
        self.current_pose = None

    def poseCallback(self, current_pose):
        self.current_pose = current_pose
 
    def update(self, state, a, delta):
        if self.current_pose is None:
            rospy.logwarn('Pose is not yet published waiting ...')
            while self.current_pose is None:
                pass
        if delta >= self.max_steer:
            delta = self.max_steer
        if delta <= - self.max_steer:
            delta = - self.max_steer
        yaw = euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])[-1]
        state.x = -self.current_pose.position.x
        state.y = self.current_pose.position.y
        state.yaw = yaw
        state.v = state.v + a * self.dt
        return state

    # def update(self, state, a, delta):
    #     if delta >= self.max_steer:
    #         delta = self.max_steer
    #     if delta <= - self.max_steer:
    #         delta = - self.max_steer
    #     state.x = state.x + state.v * math.cos(state.yaw) * self.dt
    #     state.y = state.y + state.v * math.sin(state.yaw) * self.dt
    #     state.yaw = state.yaw + state.v / self.L * math.tan(delta) * self.dt
    #     state.v = state.v + a * self.dt
    #     return state

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def calc_speed_profile(self, cyaw, target_speed):
        speed_profile = [target_speed] * len(cyaw)

        direction = 1.0

        # Set stop point
        for i in range(len(cyaw) - 1):
            dyaw = abs(cyaw[i + 1] - cyaw[i])
            switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

            if switch:
                direction *= -1

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

            if switch:
                speed_profile[i] = 0.0

        # speed down
        for i in range(40):
            speed_profile[-i] = target_speed / (50 - i)
            if speed_profile[-i] <= 1.0 / 3.6:
                speed_profile[-i] = 1.0 / 3.6

        return speed_profile


    def calc_speed_steer_profile(self, cx, cy, cyaw, target_speed):
        speed_profile = [target_speed] * len(cx)

        direction = 1.0

        # Set stop point
        for i in range(len(cx) - 1):
            dyaw = abs(cyaw[i + 1] - cyaw[i])
            switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

            if switch:
                direction *= -1

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

            if switch:
                speed_profile[i] = 0.0

        speed_profile[-1] = 0.0

        return speed_profile

    def calc_nearest_index(self, state, cx, cy, cyaw):
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind)

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = self.pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind

    def solve_dare(self, A, B, Q, R):
        """
        solve a discrete time_Algebraic Riccati equation (DARE)
        """
        x = Q
        x_next = Q
        max_iter = 150
        eps = 0.01
        
        for _ in range(max_iter):
            a = np.dot(np.dot(A.T, x), A)
            b = np.dot(np.dot(A.T, x), B)
            c = la.inv(R + np.dot(np.dot(B.T, x), B))
            d = np.dot(np.dot(B.T, x), A)
            e = np.dot(np.dot(b, c), d)
            x_next = a - e + Q      
            if (abs(x_next - x)).max() < eps:
                break
            x = x_next

        return x_next


    def dlqr(self, A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_dare(A, B, Q, R)

        # compute the LQR gain
        a = np.dot(np.dot(B.T, X), B)
        b = np.dot(np.dot(B.T, X), A)
        c = la.inv(a + R)
        d = np.dot(c, b)
        K = d

        eig_result = la.eig(A - np.dot(B, K))

        return K, X, eig_result[0]

class LqrController(object):
    def __init__(self):
        self.utils = Utils()

    def lqr_speed_steering_control(self, state, cx, cy, cyaw, ck, pe, pth_e, sp, Q=np.eye(5), R=np.eye(2)):
        ind, e = self.utils.calc_nearest_index(state, cx, cy, cyaw)

        tv = sp[ind]

        k = ck[ind]
        v = state.v
        th_e = self.utils.pi_2_pi(state.yaw - cyaw[ind])

        # A = [1.0, dt, 0.0, 0.0, 0.0
        #      0.0, 0.0, v, 0.0, 0.0]
        #      0.0, 0.0, 1.0, dt, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 1.0]
        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = self.utils.dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = self.utils.dt
        A[4, 4] = 1.0

        # B = [0.0, 0.0
        #     0.0, 0.0
        #     0.0, 0.0
        #     v/L, 0.0
        #     0.0, dt]
        B = np.zeros((5, 2))
        B[3, 0] = v / self.utils.L
        B[4, 1] = self.utils.dt

        K, _, _ = self.utils.dlqr(A, B, Q, R)

        # state vector
        # x = [e, dot_e, th_e, dot_th_e, delta_v]
        # e: lateral distance to the path
        # dot_e: derivative of e
        # th_e: angle difference to the path
        # dot_th_e: derivative of th_e
        # delta_v: difference between current speed and target speed
        x = np.zeros((5, 1))
        x[0, 0] = e
        x[1, 0] = (e - pe) / self.utils.dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - pth_e) / self.utils.dt
        x[4, 0] = v - tv

        # input vector
        # u = [delta, accel]
        # delta: steering angle
        # accel: acceleration
        ustar = np.dot(-K, x)

        # calc steering input
        ff = math.atan2(self.utils.L * k, 1)  # feedforward steering angle
        fb = self.utils.pi_2_pi(ustar[0, 0])  # feedback steering angle
        delta = ff + fb

        # calc accel input
        accel = ustar[1, 0]

        return delta, ind, e, th_e, accel