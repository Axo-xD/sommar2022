import rospy
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion
from numpy import mod, pi
from math import atan2, sqrt
pose = None

def euclidean_distance(start_pose, goal_pose):
    return sqrt(pow((goal_pose[0] - start_pose.position.x), 2) +
                pow((goal_pose[1] - start_pose.position.y), 2))

def callback(data):
    global pose
    pose = data
rospy.init_node("test")
rospy.Subscriber('/pose', Pose, callback)
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(1)
while pose == None and not rospy.is_shutdown():
    pass

dx = -4
dy = 0
start = [pose.position.x, pose.position.y]
goal = [pose.position.x + dx, pose.position.y + dy]

yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[-1]
angle = atan2(goal[1] - start[1], goal[0] - start[0])
angle = mod(angle, 2*pi)

print("Start: {}".format(start))
print("Goal: {}".format(goal))
print("Robot heading: {}".format(yaw))
print("Target angle: {}".format(angle))

vel = Twist()
alpha = abs(yaw - angle) 
while alpha > 0.05 and not rospy.is_shutdown():
    yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[-1]
    angle = atan2(goal[1] - start[1], goal[0] - start[0])
    alpha = abs(yaw - angle) 
    vel.angular.z = alpha
    cmd_pub.publish(vel)
dist = euclidean_distance(pose, goal)
vel = Twist()
while dist > 0.6 and not rospy.is_shutdown():
    yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[-1]
    angle = atan2(goal[1] - start[1], goal[0] - start[0])
    alpha = abs(yaw - angle) 
    dist = euclidean_distance(pose, goal)
    vel.linear.x = 0.5*dist
    vel.angular.z = alpha
    cmd_pub.publish(vel)
print(dist)

# # yaw = mod(yaw, 2*pi)
# # angle = mod(angle, 2*pi)
# yaw = yaw
# angle = angle

# rospy.logerr("After execution")
# print("Robot heading: {}".format(yaw))
# print("Target angle: {}".format(angle))