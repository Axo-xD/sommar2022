import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix
import tf
from mbs_msgs.srv import GPStoMAP, GPStoMAPRequest
import geonav_transform.geonav_conversions as gc
reload(gc)

def utmToOdom(utm):
    transform = False
    target_point = None
    while not transform:
        try:
            listener.waitForTransform("odom", "utm", rospy.Time.now(), rospy.Duration(3.0))
            target_point = listener.transformPoint("odom", utm)
            transform = True
        except tf.Exception as e:
            continue
    return target_point

def latLonToUTM(lat, lon):
    utmx, utmy, _ = gc.LLtoUTM(lat, lon)
    utm_point = PointStamped()
    utm_point.header.stamp = rospy.Time.now()
    utm_point.header.frame_id = "utm"
    utm_point.point.x = utmx
    utm_point.point.y = utmy
    return utm_point

def latLonToXY(lat, lon):
    gps = rospy.wait_for_message("mbs_husky/gps/filtered", NavSatFix)       
    w_x, w_y = gc.ll2xy(lat, lon, gps.latitude, gps.longitude)
    xy_point = PointStamped()
    xy_point.header.stamp = rospy.Time.now()
    xy_point.header.frame_id = "map"
    xy_point.point.x = w_x
    xy_point.point.y = w_y
    return xy_point

def pointConversions(lat, lon):
    gps = rospy.wait_for_message("mbs_husky/gps/filtered", NavSatFix)       

    # start_utm_point  = latLonToUTM(gps.latitude, gps.longitude) 
    # goal_utm_point   = latLonToUTM(lat, lon)

    # rospy.loginfo("".format())
    # rospy.logwarn("UTM: Start x, y: {}, {}".format(start_utm_point.point.x, start_utm_point.point.y))
    # rospy.logwarn("UTM: Goal  x, y: {}, {}".format(goal_utm_point.point.x, goal_utm_point.point.y))
    # rospy.loginfo("".format())

    # rospy.loginfo("".format())
    # rospy.logwarn("UTM difference: Start x, y: {}, {}".format(goal_utm_point.point.x - start_utm_point.point.x, goal_utm_point.point.y - start_utm_point.point.y))
    # rospy.loginfo("".format())

    # start_map_point  = utmToOdom(start_utm_point)
    # goal_map_point   = utmToOdom(goal_utm_point)

    # rospy.loginfo("".format())
    # rospy.logwarn("utm -> map: Start x, y: {}, {}".format(start_map_point.point.x, start_map_point.point.y))
    # rospy.logwarn("utm -> map: Goal  x, y: {}, {}".format(goal_map_point.point.x, goal_map_point.point.y))
    # rospy.loginfo("".format())


    # start_xy_point  = latLonToXY(gps.latitude, gps.longitude)
    # goal_xy_point   = latLonToXY(lat, lon)

    # rospy.loginfo("".format())
    # rospy.logwarn("(lat, lon) -> map(x, y): Start x, y: {}, {}".format(start_xy_point.point.x, start_xy_point.point.y))
    # rospy.logwarn("(lat, lon) -> map(x, y): Goal  x, y: {}, {}".format(goal_xy_point.point.x, goal_xy_point.point.y))
    # rospy.loginfo("".format())

    gps_msg = GPStoMAPRequest()
    gps_msg.gps.latitude = gps.latitude
    gps_msg.gps.longitude = gps.longitude
    gps_msg.dx = 1.0
    gps_to_map_server = rospy.ServiceProxy('gps_to_map', GPStoMAP)
    gps_res = gps_to_map_server(gps_msg)

    rospy.loginfo("".format())
    rospy.loginfo("".format())
    rospy.logwarn("Service Server Results")
    rospy.loginfo("".format())
    rospy.loginfo("".format())


    rospy.loginfo("".format())
    rospy.logwarn("(lat, lon) -> utm(x, y): {}, {}".format(gps_res.utm.point.x, gps_res.utm.point.y))
    rospy.logwarn("(lat, lon) -> map(x, y): {}, {}".format(gps_res.map.point.x, gps_res.map.point.y))
    rospy.loginfo("".format())

    # return goal_utm_point, goal_map_point, goal_xy_point

lat, lon = 50.9535944167, 6.60166041168
rospy.init_node("test")
listener = tf.TransformListener()
pointConversions(lat, lon)