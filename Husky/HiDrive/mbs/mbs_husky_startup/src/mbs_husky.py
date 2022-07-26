import rospy, roslaunch, os, rospkg


husky_base = os.path.join(rospkg.RosPack().get_path("mbs_husky_startup"), 'launch', 'husky_base.launch')
ouster_driver = os.path.join(rospkg.RosPack().get_path("mbs_husky_ouster"), 'launch', 'ouster.launch')
ouster_laserscan = os.path.join(rospkg.RosPack().get_path("mbs_husky_utilities"), 'launch', 'laserscan.launch')

rospy.init_node('mbs_husky_system_launch', anonymous=False)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

husky_base = ['mbs_husky_startup', 'husky_base.launch']
ouster_driver = ['mbs_husky_ouster', 'ouster.launch']
ouster_laserscan = ['mbs_husky_utilities', 'laserscan.launch']


roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(ouster_driver)
roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(ouster_laserscan)
roslaunch_file3 = roslaunch.rlutil.resolve_launch_arguments(husky_base)

launch_files = [(roslaunch_file1, []), (roslaunch_file2, []), (roslaunch_file3, [])]

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

parent.start()