import rospy
import time
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix
import geonav_transform.geonav_conversions as gc
reload(gc)

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4  

class GPSMoveBaseClient(object):      
    def __init__(self, action_server_name = '/move_base'):
        # initializes the action client node
        rospy.init_node('move_base_gps_node')
        rospy.Subscriber("/emlid/fix", NavSatFix, self.gpsCalback)
        self.client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
        rospy.loginfo('Waiting for action Server ' + action_server_name)
        self.client.wait_for_server()
        rospy.loginfo('Action Server Found ... ' + action_server_name)

        self.listener = tf.TransformListener()
        self.c_lat = 0.0
        self.c_lon = 0.0

    def gpsCalback(self, gps):
        self.c_lat = gps.latitude
        self.c_lon = gps.longitude

    def moveBaseCallback(self, feedback):
        pass

    def utmToOdom(self, utm):
        transform = False
        target_point = None
        while not transform:
            try:
                self.listener.waitForTransform("odom", "utm", rospy.Time.now(), rospy.Duration(3.0))
                target_point = self.listener.transformPoint("odom", utm)
                transform = True
            except tf.Exception as e:
                continue
        return target_point

    def latLonToUTM(self, lat, lon):
        utmy, utmx, utmzone = gc.LLtoUTM(lat, lon)
        utm_point = PointStamped()
        utm_point.header.stamp = rospy.Time.now()
        utm_point.header.frame_id = "utm"
        utm_point.point.x = utmx
        utm_point.point.y = utmy
        return utm_point

    def latLonToXY(self, lat, lon):
        gps = rospy.wait_for_message("/gps/filtered", NavSatFix)       
        w_x, w_y = gc.ll2xy(lat, lon, gps.latitude, gps.longitude)
        xy_point = PointStamped()
        xy_point.header.stamp = rospy.Time.now()
        xy_point.header.frame_id = "map"
        xy_point.point.x = w_x
        xy_point.point.y = w_y
        return xy_point

    def createMoveBaseGoal(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = point.point.x
        goal.target_pose.pose.position.y = point.point.y
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        return goal

    def pointConversions(self, lat, lon):
        gps = rospy.wait_for_message("/gps/filtered", NavSatFix)       

        start_utm_point  = self.latLonToUTM(gps.latitude, gps.longitude)
        goal_utm_point   = self.latLonToUTM(lat, lon)

        rospy.loginfo("".format())
        rospy.logwarn("UTM: Start x, y: {}, {}".format(start_utm_point.point.x, start_utm_point.point.y))
        rospy.logwarn("UTM: Goal  x, y: {}, {}".format(goal_utm_point.point.x, goal_utm_point.point.y))
        rospy.loginfo("".format())

        rospy.loginfo("".format())
        rospy.logwarn("UTM difference: Start x, y: {}, {}".format(goal_utm_point.point.x - start_utm_point.point.x, goal_utm_point.point.y - start_utm_point.point.y))
        rospy.loginfo("".format())

        start_map_point  = self.utmToOdom(start_utm_point)
        goal_map_point   = self.utmToOdom(goal_utm_point)

        rospy.loginfo("".format())
        rospy.logwarn("utm -> map: Start x, y: {}, {}".format(start_map_point.point.x, start_map_point.point.y))
        rospy.logwarn("utm -> map: Goal  x, y: {}, {}".format(goal_map_point.point.x, goal_map_point.point.y))
        rospy.loginfo("".format())


        start_xy_point  = self.latLonToXY(gps.latitude, gps.longitude)
        goal_xy_point   = self.latLonToXY(lat, lon)

        rospy.loginfo("".format())
        rospy.logwarn("(lat, lon) -> map(x, y): Start x, y: {}, {}".format(start_xy_point.point.x, start_xy_point.point.y))
        rospy.logwarn("(lat, lon) -> map(x, y): Goal  x, y: {}, {}".format(goal_xy_point.point.x, goal_xy_point.point.y))
        rospy.loginfo("".format())

        return goal_utm_point, goal_map_point, goal_xy_point


    def sendMoveBaseGoal(self):
        lat, lon,  = 50.9535405663, 6.60173676025
        # utm_point  = self.latLonToUTM(lat, lon)
        # map_point  = self.utmToOdom(utm_point)
        # xy_point   = self.latLonToXY(lat, lon)
        goal_utm_point, goal_map_point, goal_xy_point = self.pointConversions(lat, lon)
        goal = self.createMoveBaseGoal(goal_map_point)
        self.client.send_goal(goal, feedback_cb=self.moveBaseCallback)
        state_result = self.client.get_state()
        rate = rospy.Rate(1)
        rospy.loginfo("state_result: " + str(state_result))
        while state_result < DONE and not rospy.is_shutdown():
            rospy.loginfo("Doing Stuff while waiting for the Server to give a result ....")
            rate.sleep()
            state_result = self.client.get_state()
            rospy.loginfo("state_result: " + str(state_result))
        rospy.loginfo("[Result] State: " + str(state_result))
        if state_result == ERROR:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == WARN:
            rospy.logwarn("There is a warning in the Server Side")
        rospy.loginfo("[Result] State: " + str(self.client.get_result()))

def main():
    gps_move_base_client = GPSMoveBaseClient()
    gps_move_base_client.sendMoveBaseGoal()

if __name__ == "__main__":
    main()