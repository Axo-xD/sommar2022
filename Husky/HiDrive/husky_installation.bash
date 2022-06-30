#!/usr/bin/env bash

function color_echo () {
    echo "$(tput setaf 1)$1$(tput sgr0)"
}

function get_package_name () {
    IFS='/'
    package=($1)
    echo ${package[-1]}
}

function git_package_install () {
    # arg1: url for the clone package.
    # arg2: installation directory for the package.
    # arg3: True if its a ROS package.    
    # arg4: True if a non ROS package needs to be installed systemwide.        
    p_name=$(get_package_name $2)    
    if [ $# -eq 0 ]; then
        color_echo "No arguments specified!"
    else
        if [ ! -d $PWD/$2 ]; then
            mkdir -p $2
        fi  
        color_echo "Cloning $p_name package in $2"                
        $1 $2
        if [ $3 != true ]; then
            cd $2 && mkdir build && cd build 
            if [$build_release == true ]; then
                cmake -DCMAKE_BUILD_TYPE=Release .. && make
            fi
            cmake .. && make
            if [ $4 == true ]; then                 
                color_echo "Installing $p_name package systemwide."                          
                sudo make install
            fi            
        fi
        cd $path/$ws
    fi
}

function install_binary_packages () {
    color_echo "Installing ros packages from binaries via rosdep."
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y    
    sudo apt install ros-$ROS_DISTRO-husky*
    sudo apt install ros-$ROS_DISTRO-geodesy ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-nmea-msgs ros-$ROS_DISTRO-libg2o
    color_echo "Installing build pacakges."
    sudo apt install python-catkin-tools libpcl-dev build-essential cmake libglfw3-dev libglew-dev libeigen3-dev libjsoncpp-dev libtclap-dev libeigen3-dev
}

# if [[ $EUID -ne 0 ]]; then
#    color_echo "This script must be run as root" 
#    color_echo "sudo ./husky_installation.bash"
#    exit 1
# fi

c_dir=$PWD
build_release=false
source /opt/ros/melodic/setup.bash
read -p "Enter workspace name: " ws
read -p "Enter workspace path(Enter full path): " path

echo ""
echo "[Note] This script is to build ROS and non-ROS pacakges and their dependencies for  MBS - RWTH - HUSKY Project."
echo "[Note] This does not install ROS, for its installation please run the other script."
echo "[Note] Target OS version                          >>> Ubuntu 18.04.x (Bionic Beaver) or Linux Mint 19.x"
echo "[Note] Target ROS version                         >>> ROS Melodic Morenia"
echo "[Note] Catkin workspace                           >>> $path/$ws"
echo "[Note] Non-ROS package dependencies built space:  >>> $path/$ws/utils"
echo "[Note] Robot specific package's built space:      >>> $path/$ws/src/mbs"
echo "[Note] Package dependencies built space:          >>> $path/$ws/src/third_party"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

if [ ! -d $path/$ws ]; then
  mkdir -p $path/$ws/src $path/$ws/utils
  cd $path/$ws/src && catkin_init_workspace && cd $c_dir
fi

cd $path/$ws

# Lets start by cloning ros packages first

# hdl_slam and its dependencies
git_package_install "git clone https://github.com/koide3/hdl_graph_slam" src/third_party/3d_slam/hdl_graph_slam true
git_package_install "git clone https://github.com/koide3/ndt_omp.git -b melodic" src/third_party/common_packages/ndt_omp true
git_package_install "git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive" src/third_party/common_packages/fast_gicp true

# hdl_localization and its dependencies
git_package_install "git clone https://github.com/koide3/hdl_localization" src/third_party/3d_localization/hdl_localization true
git_package_install "git clone https://github.com/koide3/hdl_global_localization" src/third_party/common_packages/hdl_global_localization true

# hdl_people_tracking
git_package_install "git clone https://github.com/koide3/hdl_people_tracking" src/third_party/3d_people_tracking/hdl_people_tracking true

# Outdoor waypoint navigation
git_package_install "git clone https://github.com/nickcharron/waypoint_nav" src/third_party/waypoint_navigation/waypoint_nav true

# Building non-ROS packages
# Ouster driver
build_release=true
git_package_install "git clone https://github.com/ouster-lidar/ouster_example" utils/ouster_driver/ouster_example false false
build_release=false
# Symlink for Ouster ROS driver
sudo ln -s $PWD/utils/ouster_driver/ouster_example $PWD/src/third_party

# Emlid ROS driver
git_package_install "git clone https://github.com/enwaytech/reach_rs_ros_driver" src/third_party/emlid_ros_driver true

# Geographic Lib
git_package_install "git clone git://git.code.sourceforge.net/p/geographiclib/code" utils/geographiclib false true

# Copy Husky packages and some fixed
cp -r $c_dir/mbs src/
cp -f $c_dir/utils/CMakeLists.txt src/third_party/waypoint_navigation/waypoint_nav/outdoor_waypoint_nav
rm -rf src/third_party/waypoint_navigation/waypoint_nav/h* slam_gmapping/ # Lets remove these unnecessary pacakges, we will install later on from binaries if needed

# Binary packages installation
install_binary_packages

# Lets build everything for ROS
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
