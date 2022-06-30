import rospy
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion
from numpy import mod, pi
from math import atan2, sqrt, tanh
pose = Pose()
received = False

def euclidean_distance(start_pose, goal_pose):
    return sqrt(pow((goal_pose[0] - start_pose.position.x), 2) +
                pow((goal_pose[1] - start_pose.position.y), 2))

def callback(data): 
    global received
    received = True   
    pose.position.x = -1.0*round(data.position.x, 6)
    pose.position.y = round(data.position.y, 6)

    pose.orientation.x = round(data.orientation.x, 6)
    pose.orientation.y = round(data.orientation.y, 6) 
    pose.orientation.z = round(data.orientation.z, 6)
    pose.orientation.w = round(data.orientation.w, 6) 

rospy.init_node("test")
rospy.Subscriber('/pose', Pose, callback)
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(1)
while not received and not rospy.is_shutdown():
    pass

dx = -1
dy = 0
start = [pose.position.x, pose.position.y]
goal  = [pose.position.x + dx, pose.position.y + dy]

yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[-1]
angle = atan2(goal[1] - start[1], goal[0] - start[0])
# yaw = mod(euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[-1], 2*pi)
# angle = mod(atan2(goal[1] - start[1], goal[0] - start[0]), 2*pi)

print("Start: {}".format(start))
print("Goal: {}".format(goal))
print("Robot heading: {}".format(yaw))
print("Target angle: {}".format(angle))

vel = Twist()
alpha = angle - yaw 
print(alpha)
while abs(alpha) > 0.05 and not rospy.is_shutdown():
    yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[-1]
    alpha = angle - yaw 
    vel.angular.z = tanh(alpha)
    cmd_pub.publish(vel)
print(alpha)
# input()
dist = euclidean_distance(pose, goal)
vel = Twist()
while dist > 0.4 and not rospy.is_shutdown():
    yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[-1]
    angle = atan2(goal[1] - start[1], goal[0] - start[0])
    alpha = abs(angle - yaw) 
    dist = euclidean_distance(pose, goal)
    vel.linear.x = tanh(dist)
    # vel.angular.z = tanh(alpha)
    # cmd_pub.publish(vel)
    # print("Distance between current {} and goal point {} is {}.".format([pose.position.x, pose.position.y], goal, dist))

# # yaw = mod(yaw, 2*pi)
# # angle = mod(angle, 2*pi)
# yaw = yaw
# angle = angle

# rospy.logerr("After execution")
# print("Robot heading: {}".format(yaw))
# print("Target angle: {}".format(angle))