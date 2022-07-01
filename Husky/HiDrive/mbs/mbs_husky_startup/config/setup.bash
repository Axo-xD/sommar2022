# Start from sourcing the workspoaces
if [ $(whoami) == "root" ] 
then
    source /opt/ros/$ROS_DISTRO/setup.bash
    source /home/administrator/catkin_ws/devel/setup.bash
fi
# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=$(rospack find mbs_husky_startup)/config/setup.bash
# Setup robot upstart jobs to use the IP from the network bridge.
export ROBOT_NETWORK=enp1s0

# Insert extra platform-level environment variables here. The six hashes below are a marker
# for scripts to insert to this file.
######
export HUSKY_LOGITECH=1

# export HUSKY_NAVSAT_PORT=/dev/clearpath/gps
# export HUSKY_NAVSAT_BAUD=19200

# export HUSKY_IMU_XYZ='0.15 -0.09 0.08'
export HUSKY_IMU_RPY='0 0 1.5708'

# Changed Husky URDF to include MYBOTSHOP updates.
export HUSKY_URDF_EXTRAS=$(rospack find mbs_husky_description)/urdf/mbs_husky_updates.urdf.xacro

# We don't want to start EKF Localization, this will be handled seperately with navigation.
export ENABLE_EKF=true
