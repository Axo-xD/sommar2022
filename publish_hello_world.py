#!/usr/bin/python3

import rospy
from std_msgs.msg import String
def talker():

    pub = rospy.Publisher('Testchannel', String, queue_size=10)
    rospy.init_node('Helloman', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        string_1 = "hello world, my time is %s" % rospy.get_time()
        rospy.loginfo("this is my message: %s" % string_1)
        pub.publish(string_1)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
