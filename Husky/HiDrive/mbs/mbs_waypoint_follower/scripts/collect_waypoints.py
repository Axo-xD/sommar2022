#!/usr/bin/env python

from math import sqrt, pow
import yaml
import rospy, os, rospkg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy

class CollectWaypoints(object):
    def __init__(self, waypoint_source_topic, waypoint_source_type):
        self.collect_button_num = rospy.get_param('~collect_button_num', 3)
        self.collect_button_sym = rospy.get_param('~collect_button_sym', 'y')
        self.end_button_num = rospy.get_param('~end_button_num', 6)
        self.end_button_sym = rospy.get_param('~end_button_sym', 'back')
        self.default_waypoint_file = os.path.join(rospkg.RosPack().get_path('mbs_waypoint_follower'), 'waypoints', 'waypoints.yaml') 
        self.file_path = rospy.get_param('~waypoints_file_path', self.default_waypoint_file)
        self.distance_threshold = rospy.get_param('~waypoint_distance', 2)
        rospy.Subscriber('joy_teleop/joy', Joy, self.joyCallback)   

        self.waypoint_source_topic = waypoint_source_topic
        self.waypoint_source_type = waypoint_source_type
        self.collect_request = False
        self.continue_collection = True
        self.last_point = []
        self.last_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.duration = rospy.Duration(1)
        self.rate = rospy.Rate(10)

        # Data objects to store the data
        self.frame_id = None
        self.positions = []
        self.orientations = []

    def initializeWayPointCollector(self):
        try:
            waypoint = rospy.wait_for_message(self.waypoint_source_topic, self.waypoint_source_type, 3)
            self.last_point, _ = self.extractPoint(waypoint)
            rospy.loginfo("Initialized waypoint collector. Recording waypoints from topic {}.".format(self.waypoint_source_topic))
            self.usage()
        except rospy.ROSException:
            rospy.logerr("Waypoint collector initialization failed. Message not recveived on topic {}.".format(self.waypoint_source_topic))   
            raise rospy.ROSException

    def joyCallback(self, joy_msg):
        if(joy_msg.buttons[self.collect_button_num]==1):
            self.collect_request = True        
        else:
            self.collect_request = False
        if(joy_msg.buttons[self.end_button_num]==1):	
            self.continue_collection = False

    def extractPoint(self, data):
        if data._type in ['nav_msgs/Odometry', 'geometry_msgs/PoseWithCovarianceStamped']:     
        # if self.waypoint_source_type == Odometry:
            position = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
            orientation = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
            if self.frame_id is None:
                self.frame_id = data.header.frame_id
        return position, orientation

    def writeYAML(self):
        waypoints_object = {}
        for i in range(len(self.positions)):
            waypoints_object.update({i+1: {
                                        'position': self.positions[i], 
                                        'orientation': self.orientations[i]
                                        # Here add any other disctionary object for example for 5g points intensity etc for each waypoint.
                                        }
                                    })
        waypoints = {'frame_id': self.frame_id}
        waypoints.update({'waypoints': waypoints_object})
        with open(self.file_path, 'w') as waypoints_file:
            yaml.dump(waypoints, waypoints_file)

    def euclideanDistance(self, goal_pose):
        return sqrt(pow((goal_pose[0] - self.last_point[0]), 2) +
                    pow((goal_pose[1] - self.last_point[1]), 2) +
                    pow((goal_pose[2] - self.last_point[2]), 2))
    
    def usage(self):
        rospy.loginfo("Press {} button to collect and store waypoint.".format(self.collect_button_sym))
        rospy.loginfo("Press {} button to end waypoint collection.".format(self.end_button_sym))

    def recordWaypoints(self):
        self.initializeWayPointCollector()
        while not rospy.is_shutdown() and self.continue_collection:
            self.current_time = rospy.Time.now()
            if self.collect_request == True and ((self.current_time - self.last_time) > self.duration):
                try:
                    waypoint = rospy.wait_for_message(self.waypoint_source_topic, self.waypoint_source_type, 3)
                    position, orientation = self.extractPoint(waypoint)
                    distance = self.euclideanDistance(position)
                    rospy.logwarn(distance)
                    if  distance > self.distance_threshold or self.positions == []:
                        self.positions.append(position)
                        self.orientations.append(orientation)
                        self.last_point = position
                        rospy.loginfo("New waypoint {} saved.".format(position))
                        self.usage()
                    else:
                        rospy.logwarn("Waypoint not saved, you have not moved enough.")
                        rospy.logwarn("Last point: {}.".format(self.last_point))
                        rospy.logwarn("Current point: {}.".format(position))
                        self.usage()
                    self.last_time = self.current_time
                except rospy.ROSException:
                    pass  
            else:
                self.rate.sleep()
        self.writeYAML()

def main():
    rospy.init_node("waypoints_collection", anonymous=False)
    waypoints_collector = CollectWaypoints('amcl_pose', PoseWithCovarianceStamped)
    waypoints_collector.recordWaypoints()

if __name__ == '__main__':
    main()