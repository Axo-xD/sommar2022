#!/usr/bin/env python
#license: MIT

import enum
import rospy
from geometry_msgs.msg import Twist

def talker():
    # V2 additions:
    counter = 0

    # V1:
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('turtleDriver', anonymous=False)
    rate = rospy.Rate(1) # X amount of Hz
    while not rospy.is_shutdown():
        # V2:
        if counter is not 1:
            velocity = Twist()
            velocity.linear.x = 2.0
            rospy.loginfo(velocity)
            pub.publish(velocity)
            counter = 1
            rate.sleep()
        else:
            velocity = Twist()
            velocity.angular.z = 1.6
            rospy.loginfo(velocity)
            pub.publish(velocity)
            counter = 0
            rate.sleep()

        # V1:
        #velocity = Twist()
        #velocity.linear.x = 2.0
        #velocity.angular.z = -1.8
        #rospy.loginfo(velocity)
        #pub.publish(velocity)
        #rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass