#!/bin/bash
export DISPLAY=:0
# export ROS_MASTER_URI=http://10.42.0.1:11311
sudo modprobe denso_cobotta io_power_mode=1
sudo chown cobotta:cobotta /dev/denso_cobotta
# xterm -e "bash -c roslaunch denso_cobotta_bringup denso_cobotta_bringup.launch" &
# xterm -e "bash -c sleep 5; python ./soma.py" &  # script for Cobotta's grasping
roslaunch denso_cobotta_bringup denso_cobotta_bringup.launch
