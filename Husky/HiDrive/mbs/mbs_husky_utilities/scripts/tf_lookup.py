#!/usr/bin/env python

import tf
import rospy

if __name__ == '__main__':
    rospy.init_node('tf_lookup')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/utm', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            rospy.loginfo("odom to base_link translation is: %s and the rotation is: %s.", [round(t, 3)for t in trans[0:2]], 
                                                                                           [round(e, 3) for e in euler])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()