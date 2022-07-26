import rospy
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from numpy import degrees
angle = 0
def callback(data):
    global angle
    angle = degrees(euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])[-1])
    # rospy.loginfo("Linear: {}, angular: {}.".format([data.position.x, data.position.y], angle)

rospy.init_node('test')
rospy.Subscriber('pose', Pose, callback)
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    rospy.loginfo("Angle: {}.".format(angle))
    rate.sleep()