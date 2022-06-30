#!/usr/bin/env bash

source /etc/ros/setup.bash
sudo systemctl stop mbs_husky.service
rosrun robot_upstart uninstall mbs_husky
rosrun robot_upstart install mbs_husky_startup/launch/mbs_husky.launch --job mbs_husky
sudo systemctl daemon-reload && sudo systemctl start mbs_husky