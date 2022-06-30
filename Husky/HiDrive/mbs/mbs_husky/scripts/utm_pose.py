#!/usr/bin/env python
import message_filters
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import NavSatFix
import geonav_transform.geonav_conversions as gc
reload(gc)

class UtmPose(object):
    def __init__(self, name, q_size, slop, gps_topic='/navsat/fix', imu_topic='imu/data'):
        self.name = name
        self.q_size = q_size
        self.slop = slop
        self.status = 0
        self.gps_topic = gps_topic
        self.imu_topic = imu_topic
        self.utm_pose = Pose()
        self.pose_pub = rospy.Publisher('pose', Pose, queue_size=10)
        self.set_precision = lambda data, p=6: round(data, p)
    
    def latLonToUTM(self, lat, lon):
        utmx, utmy, _ = gc.LLtoUTM(lat, lon)
        utm_point = Point()
        utm_point.x = self.set_precision(utmx)
        utm_point.y = self.set_precision(utmy)
        return utm_point

    def dataCallback(self, gps, imu):
        utm_point = self.latLonToUTM(gps.latitude, gps.longitude)
        # self.utm_pose.header.stamp = rospy.Time.now()
        # self.utm_pose.header.frame_id = 'utm'
        self.utm_pose.position = utm_point 
        self.utm_pose.orientation = imu.orientation

    def gpsToUtmPoseNode(self):
        rospy.init_node(self.name, anonymous = False)
        rospy.loginfo("Started node: {}".format(self.name))
        gps_sub = message_filters.Subscriber(self.gps_topic, NavSatFix)
        imu_sub = message_filters.Subscriber(self.imu_topic, Imu)
        filter = message_filters.ApproximateTimeSynchronizer([gps_sub, imu_sub], self.q_size, self.slop, allow_headerless=True)
        filter.registerCallback(self.dataCallback)
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            self.pose_pub.publish(self.utm_pose)
            rate.sleep()

if __name__ == '__main__':
    utm = UtmPose("utm_pose", 10, 0.1)
    utm.gpsToUtmPoseNode()