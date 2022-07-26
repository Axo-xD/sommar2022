#!/usr/bin/env bash
echo "This script will update the robot upstart jobs on Husky."
read -p "If you want to update press Enter or press CTRL+C to cancel" var
if [ ${#var} -eq 0 ]; then
    source $(rospack find mbs_husky_startup)/config/setup.bash
    sudo systemctl stop mbs_husky.service
    rosrun robot_upstart uninstall mbs_husky    
    rosrun robot_upstart install mbs_husky_startup/launch/mbs_husky.launch --job mbs_husky --setup $(rospack find mbs_husky_startup)/config/setup.bash
    sudo systemctl daemon-reload && sudo systemctl start mbs_husky
fi